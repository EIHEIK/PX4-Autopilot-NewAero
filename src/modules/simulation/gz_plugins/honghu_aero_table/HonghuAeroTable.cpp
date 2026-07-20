/****************************************************************************
 * Honghu lookup-table aerodynamic plugin for Gazebo SITL.
 ****************************************************************************/

#include "HonghuAeroTable.hpp"

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>

#include <sdf/Element.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace gz;
using namespace gz::sim;

namespace custom
{
namespace
{
constexpr double kEpsilon = 1e-6;
constexpr double kRadToDeg = 180.0 / GZ_PI;

std::vector<std::string> SplitCsvLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::string Trim(const std::string &value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

double Clamp(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}
} // namespace

struct AeroTable
{
    std::vector<double> alpha_deg;
    std::vector<double> beta_deg;
    std::vector<std::vector<double>> values;

    bool Load(const std::filesystem::path &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            gzerr << "HonghuAeroTable: failed to open " << path << "\n";
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            line = Trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }

            const auto header = SplitCsvLine(line);
            if (header.size() < 2) {
                gzerr << "HonghuAeroTable: invalid CSV header in " << path << "\n";
                return false;
            }
            beta_deg.clear();
            for (size_t i = 1; i < header.size(); ++i) {
                beta_deg.push_back(std::stod(Trim(header[i])));
            }
            break;
        }

        while (std::getline(file, line)) {
            line = Trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }
            const auto fields = SplitCsvLine(line);
            if (fields.size() != beta_deg.size() + 1) {
                gzerr << "HonghuAeroTable: row width mismatch in " << path << ": " << line << "\n";
                return false;
            }
            alpha_deg.push_back(std::stod(Trim(fields[0])));
            std::vector<double> row;
            row.reserve(beta_deg.size());
            for (size_t i = 1; i < fields.size(); ++i) {
                row.push_back(std::stod(Trim(fields[i])));
            }
            values.push_back(row);
        }

        if (alpha_deg.size() < 2 || beta_deg.size() < 2 || values.size() != alpha_deg.size()) {
            gzerr << "HonghuAeroTable: not enough table data in " << path << "\n";
            return false;
        }
        return true;
    }

    double Interpolate(double alpha_query_deg, double beta_query_deg) const
    {
        const double alpha = Clamp(alpha_query_deg, alpha_deg.front(), alpha_deg.back());
        const double beta = Clamp(beta_query_deg, beta_deg.front(), beta_deg.back());

        auto upper_a = std::upper_bound(alpha_deg.begin(), alpha_deg.end(), alpha);
        size_t ia1 = std::distance(alpha_deg.begin(), upper_a);
        if (ia1 == 0) {
            ia1 = 1;
        } else if (ia1 >= alpha_deg.size()) {
            ia1 = alpha_deg.size() - 1;
        }
        const size_t ia0 = ia1 - 1;

        auto upper_b = std::upper_bound(beta_deg.begin(), beta_deg.end(), beta);
        size_t ib1 = std::distance(beta_deg.begin(), upper_b);
        if (ib1 == 0) {
            ib1 = 1;
        } else if (ib1 >= beta_deg.size()) {
            ib1 = beta_deg.size() - 1;
        }
        const size_t ib0 = ib1 - 1;

        const double a0 = alpha_deg[ia0];
        const double a1 = alpha_deg[ia1];
        const double b0 = beta_deg[ib0];
        const double b1 = beta_deg[ib1];
        const double ta = std::abs(a1 - a0) < kEpsilon ? 0.0 : (alpha - a0) / (a1 - a0);
        const double tb = std::abs(b1 - b0) < kEpsilon ? 0.0 : (beta - b0) / (b1 - b0);

        const double v00 = values[ia0][ib0];
        const double v10 = values[ia1][ib0];
        const double v01 = values[ia0][ib1];
        const double v11 = values[ia1][ib1];
        return (1.0 - ta) * ((1.0 - tb) * v00 + tb * v01)
             + ta * ((1.0 - tb) * v10 + tb * v11);
    }
};

struct ControlSurface
{
    Entity joint{kNullEntity};
    double direction{1.0};
    double scale{1.0};
    double ref_deflection_deg{10.0};
    std::string effect_type{"linear"};
    std::string table_prefix;

    // Legacy constant-per-degree derivatives. Kept as a fallback for old SDFs,
    // but V7 uses Word-document lookup tables instead.
    double cd{0.0};
    double cy{0.0};
    double cl_force{0.0};
    double cl_roll{0.0};
    double cm{0.0};
    double cn{0.0};

    AeroTable cl_table;
    AeroTable cd_table;
    AeroTable cy_table;
    AeroTable cl_roll_table;
    AeroTable cm_table;
    AeroTable cn_table;
    bool has_cl_table{false};
    bool has_cd_table{false};
    bool has_cy_table{false};
    bool has_cl_roll_table{false};
    bool has_cm_table{false};
    bool has_cn_table{false};
};

class HonghuAeroTablePrivate
{
public:
    bool Load(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm);
    void Update(const UpdateInfo &_info, EntityComponentManager &_ecm);

private:
    std::filesystem::path ResolveTableDirectory(const std::string &raw_path) const;
    std::filesystem::path ResolveForceLogPath(const std::filesystem::path &table_dir,
                                              const std::string &raw_path) const;
    bool LoadTables(const std::filesystem::path &table_dir);
    void LoadControlSurfaces(const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             const std::filesystem::path &table_dir);
    double GetControlAngleDeg(const ControlSurface &surface, EntityComponentManager &_ecm) const;

    Model _model{kNullEntity};
    Entity _link_entity{kNullEntity};
    std::string _link_name;
    bool _valid{false};

    AeroTable _cl_table;
    AeroTable _cd_table;
    AeroTable _cy_table;
    AeroTable _cm_table;
    AeroTable _cl_roll_table;
    AeroTable _cn_table;

    std::vector<ControlSurface> _control_surfaces;

    double _area{2.42};
    double _span{3.96};
    double _mac{0.62};
    double _rho{1.225};
    math::Vector3d _cp{math::Vector3d::Zero};
    math::Vector3d _forward{math::Vector3d::UnitX};
    math::Vector3d _upward{math::Vector3d::UnitZ};

    double _CLq{5.62};
    double _Cmq{-7.0};
    double _Clp{-0.33};
    double _Cnp{-0.05};
    double _CYp{-0.15};
    double _Cnr{-0.08};
    double _Clr{0.10};
    double _CYr{0.34};
    double _Cma{-0.33};
    double _Cnb{0.14};

    bool _debug{false};
    double _debug_rate_hz{1.0};
    double _next_debug_time_s{0.0};

    bool _force_log{true};
    double _force_log_rate_hz{20.0};
    double _next_force_log_time_s{0.0};
    std::string _force_log_path{"honghu_v5_aero_forces.csv"};
    std::ofstream _force_log_stream;
};

std::filesystem::path HonghuAeroTablePrivate::ResolveTableDirectory(const std::string &raw_path) const
{
    constexpr const char *model_prefix = "model://";
    if (raw_path.rfind(model_prefix, 0) != 0) {
        return std::filesystem::path(raw_path);
    }

    const std::string relative = raw_path.substr(std::char_traits<char>::length(model_prefix));
    const char *resource_env = std::getenv("GZ_SIM_RESOURCE_PATH");
    if (!resource_env) {
        return std::filesystem::path(raw_path);
    }

    std::stringstream resources(resource_env);
    std::string root;
    while (std::getline(resources, root, ':')) {
        if (root.empty()) {
            continue;
        }
        std::filesystem::path candidate = std::filesystem::path(root) / relative;
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }

        const auto root_path = std::filesystem::path(root);
        if (root_path.filename() == "worlds") {
            candidate = root_path.parent_path() / "models" / relative;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }

        if (root_path.filename() == "models"
            && root_path.parent_path().filename() == "gz"
            && root_path.parent_path().parent_path().filename() == "simulation"
            && root_path.parent_path().parent_path().parent_path().filename() == "Tools") {
            const auto repo_root = root_path.parent_path().parent_path().parent_path().parent_path();
            candidate = repo_root / "simulation_models" / "models" / relative;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }

    return std::filesystem::path(raw_path);
}

std::filesystem::path HonghuAeroTablePrivate::ResolveForceLogPath(
    const std::filesystem::path &table_dir, const std::string &raw_path) const
{
    std::filesystem::path log_path(raw_path);
    if (log_path.is_absolute()) {
        return log_path;
    }

    // table_dir normally points to:
    // <repo>/simulation_models/models/honghu_wing_150kg_v5/aero_tables
    // Keep relative log paths deterministic by placing them under PX4 rootfs,
    // where the user already expects SITL outputs to appear.
    for (auto current = table_dir; !current.empty(); current = current.parent_path()) {
        if (current.filename() == "simulation_models") {
            const auto repo_root = current.parent_path();
            return repo_root / "build" / "px4_sitl_default" / "rootfs" / log_path;
        }

        if (current == current.root_path()) {
            break;
        }
    }

    return std::filesystem::current_path() / log_path;
}

bool HonghuAeroTablePrivate::LoadTables(const std::filesystem::path &table_dir)
{
    return _cl_table.Load(table_dir / "CL.csv")
        && _cd_table.Load(table_dir / "CD.csv")
        && _cy_table.Load(table_dir / "CY.csv")
        && _cm_table.Load(table_dir / "Cm.csv")
        && _cl_roll_table.Load(table_dir / "Cl.csv")
        && _cn_table.Load(table_dir / "Cn.csv");
}

void HonghuAeroTablePrivate::LoadControlSurfaces(const std::shared_ptr<const sdf::Element> &_sdf,
                                                 EntityComponentManager &_ecm,
                                                 const std::filesystem::path &table_dir)
{
    const int expected_count = _sdf->Get<int>("num_ctrl_surfaces", 0).first;
    sdf::ElementPtr mutable_sdf = _sdf->Clone();
    while (mutable_sdf->HasElement("control_surface")) {
        auto surface_sdf = mutable_sdf->GetElement("control_surface");
        const auto joint_name = surface_sdf->Get<std::string>("name", "").first;
        auto entities = entitiesFromScopedName(joint_name, _ecm, _model.Entity());
        if (entities.empty()) {
            gzerr << "HonghuAeroTable: control joint [" << joint_name << "] not found\n";
            _valid = false;
            return;
        }

        ControlSurface surface;
        surface.joint = *entities.begin();
        if (!_ecm.EntityHasComponentType(surface.joint, components::Joint::typeId)) {
            gzerr << "HonghuAeroTable: [" << joint_name << "] is not a joint\n";
            _valid = false;
            return;
        }

        surface.direction = surface_sdf->Get<double>("direction", surface.direction).first;
        surface.scale = surface_sdf->Get<double>("scale", surface.scale).first;
        surface.ref_deflection_deg = surface_sdf->Get<double>("ref_deflection_deg", surface.ref_deflection_deg).first;
        surface.effect_type = surface_sdf->Get<std::string>("effect_type", surface.effect_type).first;
        surface.table_prefix = surface_sdf->Get<std::string>("table_prefix", surface.table_prefix).first;

        surface.cd = surface_sdf->Get<double>("CD_ctrl", surface.cd).first;
        surface.cy = surface_sdf->Get<double>("CY_ctrl", surface.cy).first;
        surface.cl_force = surface_sdf->Get<double>("CL_ctrl", surface.cl_force).first;
        surface.cl_roll = surface_sdf->Get<double>("Cl_ctrl", surface.cl_roll).first;
        surface.cm = surface_sdf->Get<double>("Cm_ctrl", surface.cm).first;
        surface.cn = surface_sdf->Get<double>("Cn_ctrl", surface.cn).first;

        if (!surface.table_prefix.empty() && surface.effect_type != "linear") {
            auto load_optional = [&](const char *suffix, AeroTable &table, bool &loaded) {
                const auto path = table_dir / (surface.table_prefix + "_" + suffix + ".csv");
                if (std::filesystem::exists(path)) {
                    loaded = table.Load(path);
                    if (!loaded) {
                        gzerr << "HonghuAeroTable: failed to load control table " << path << "\n";
                        _valid = false;
                    }
                }
            };

            load_optional("CL", surface.cl_table, surface.has_cl_table);
            load_optional("CD", surface.cd_table, surface.has_cd_table);
            load_optional("CY", surface.cy_table, surface.has_cy_table);
            load_optional("Cl", surface.cl_roll_table, surface.has_cl_roll_table);
            load_optional("Cm", surface.cm_table, surface.has_cm_table);
            load_optional("Cn", surface.cn_table, surface.has_cn_table);

            const bool any_table = surface.has_cl_table || surface.has_cd_table || surface.has_cy_table
                || surface.has_cl_roll_table || surface.has_cm_table || surface.has_cn_table;
            if (!any_table) {
                gzerr << "HonghuAeroTable: no control tables found for prefix "
                      << surface.table_prefix << " in " << table_dir << "\n";
                _valid = false;
                return;
            }
        }

        _control_surfaces.push_back(surface);
        mutable_sdf->RemoveChild(surface_sdf);
    }

    if (expected_count != 0 && expected_count != static_cast<int>(_control_surfaces.size())) {
        gzwarn << "HonghuAeroTable: expected " << expected_count << " control surfaces, loaded "
               << _control_surfaces.size() << "\n";
    }
}

double HonghuAeroTablePrivate::GetControlAngleDeg(const ControlSurface &surface,
                                                  EntityComponentManager &_ecm) const
{
    const auto joint_position = _ecm.Component<components::JointPosition>(surface.joint);
    if (!joint_position || joint_position->Data().empty()) {
        return 0.0;
    }
    return joint_position->Data()[0] * kRadToDeg;
}

bool HonghuAeroTablePrivate::Load(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm)
{
    _model = Model(_entity);
    if (!_model.Valid(_ecm)) {
        gzerr << "HonghuAeroTable should be attached to a model entity.\n";
        return false;
    }

    _link_name = _sdf->Get<std::string>("link_name", _link_name).first;
    if (_link_name.empty()) {
        gzerr << "HonghuAeroTable requires <link_name>.\n";
        return false;
    }

    const auto link_entities = entitiesFromScopedName(_link_name, _ecm, _model.Entity());
    if (link_entities.empty()) {
        gzerr << "HonghuAeroTable: link [" << _link_name << "] not found.\n";
        return false;
    }
    _link_entity = *link_entities.begin();
    if (!_ecm.EntityHasComponentType(_link_entity, components::Link::typeId)) {
        gzerr << "HonghuAeroTable: [" << _link_name << "] is not a link.\n";
        return false;
    }
    Link link(_link_entity);
    link.EnableVelocityChecks(_ecm, true);

    _area = _sdf->Get<double>("area", _area).first;
    _span = _sdf->Get<double>("span", _span).first;
    _mac = _sdf->Get<double>("mac", _mac).first;
    _rho = _sdf->Get<double>("air_density", _rho).first;
    _cp = _sdf->Get<math::Vector3d>("cp", _cp).first;
    _forward = _sdf->Get<math::Vector3d>("forward", _forward).first;
    _upward = _sdf->Get<math::Vector3d>("upward", _upward).first;
    _forward.Normalize();
    _upward.Normalize();

    _CLq = _sdf->Get<double>("CLq", _CLq).first;
    _Cmq = _sdf->Get<double>("Cmq", _Cmq).first;
    _Clp = _sdf->Get<double>("Clp", _Clp).first;
    _Cnp = _sdf->Get<double>("Cnp", _Cnp).first;
    _CYp = _sdf->Get<double>("CYp", _CYp).first;
    _Cnr = _sdf->Get<double>("Cnr", _Cnr).first;
    _Clr = _sdf->Get<double>("Clr", _Clr).first;
    _CYr = _sdf->Get<double>("CYr", _CYr).first;
    _Cma = _sdf->Get<double>("Cma", _Cma).first;
    _Cnb = _sdf->Get<double>("Cnb", _Cnb).first;
    _debug = _sdf->Get<bool>("debug", _debug).first;
    _debug_rate_hz = _sdf->Get<double>("debug_rate_hz", _debug_rate_hz).first;
    _force_log = _sdf->Get<bool>("force_log", _force_log).first;
    _force_log_rate_hz = _sdf->Get<double>("force_log_rate_hz", _force_log_rate_hz).first;
    _force_log_path = _sdf->Get<std::string>("force_log_path", _force_log_path).first;
    if (!_force_log_path.empty()) {
        _force_log = true;
    }

    const auto raw_table_dir = _sdf->Get<std::string>("table_directory", "").first;
    const auto table_dir = ResolveTableDirectory(raw_table_dir);
    _force_log_path = ResolveForceLogPath(table_dir, _force_log_path).string();
    if (!LoadTables(table_dir)) {
        gzerr << "HonghuAeroTable: failed to load tables from " << table_dir << "\n";
        return false;
    }
    gzmsg << "HonghuAeroTable: loaded aero tables from " << table_dir
          << " alpha=[" << _cl_table.alpha_deg.front() << ", " << _cl_table.alpha_deg.back() << "] deg"
          << " beta=[" << _cl_table.beta_deg.front() << ", " << _cl_table.beta_deg.back() << "] deg\n";
    gzmsg << "HonghuAeroTable: force_log=" << (_force_log ? "true" : "false")
          << " path=" << _force_log_path
          << " rate_hz=" << _force_log_rate_hz << "\n";

    _valid = true;
    LoadControlSurfaces(_sdf, _ecm, table_dir);

    if (_force_log && !_force_log_stream.is_open()) {
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(_force_log_path).parent_path(), ec);
        if (ec) {
            gzwarn << "HonghuAeroTable: failed to create force log directory for "
                   << _force_log_path << ": " << ec.message() << "\n";
        }

        _force_log_stream.open(_force_log_path, std::ios::out | std::ios::trunc);
        if (_force_log_stream.is_open()) {
            _force_log_stream
                << "time_s,airspeed_m_s,alpha_deg,beta_deg,p_rad_s,q_rad_s,r_rad_s,qbar_pa,"
                << "CL,CD,CY,Cl,Cm,Cn,"
                << "force_body_x_N,force_body_y_N,force_body_z_N,"
                << "moment_body_x_Nm,moment_body_y_Nm,moment_body_z_Nm,"
                << "force_world_x_N,force_world_y_N,force_world_z_N,"
                << "moment_world_x_Nm,moment_world_y_Nm,moment_world_z_Nm";

            for (size_t i = 0; i < _control_surfaces.size(); ++i) {
                _force_log_stream << ",ctrl" << i << "_deg";
            }

            _force_log_stream << "\n";
            _force_log_stream << std::setprecision(10);
            _force_log_stream.flush();
            gzmsg << "HonghuAeroTable: writing aero force log to " << _force_log_path << "\n";
        } else {
            gzwarn << "HonghuAeroTable: failed to open force log " << _force_log_path << "\n";
            _force_log = false;
        }
    }

    return _valid;
}

void HonghuAeroTablePrivate::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
    if (!_valid || _info.paused) {
        return;
    }

    const auto world_lin_vel = _ecm.Component<components::WorldLinearVelocity>(_link_entity);
    const auto world_ang_vel = _ecm.Component<components::WorldAngularVelocity>(_link_entity);
    const auto world_pose = _ecm.Component<components::WorldPose>(_link_entity);
    if (!world_lin_vel || !world_ang_vel || !world_pose) {
        if (_debug) {
            const double sim_time_s = std::chrono::duration<double>(_info.simTime).count();
            if (sim_time_s >= _next_debug_time_s) {
                _next_debug_time_s = sim_time_s + std::max(1.0 / std::max(_debug_rate_hz, 1.0), 1.0);
                gzwarn << "HonghuAeroTable: waiting for link state components:"
                       << " WorldLinearVelocity=" << (world_lin_vel ? "ok" : "missing")
                       << " WorldAngularVelocity=" << (world_ang_vel ? "ok" : "missing")
                       << " WorldPose=" << (world_pose ? "ok" : "missing") << "\n";
            }
        }
        return;
    }

    const auto &pose = world_pose->Data();
    // cp=(0,0,0): aerodynamic reference / CG — no offset correction needed.
    math::Vector3d air_velocity_world = world_lin_vel->Data();

    if (_ecm.EntityByComponents(components::Wind()) != kNullEntity) {
        const Entity wind_entity = _ecm.EntityByComponents(components::Wind());
        const auto wind_linear_vel = _ecm.Component<components::WorldLinearVelocity>(wind_entity);
        if (wind_linear_vel) {
            air_velocity_world -= wind_linear_vel->Data();
        }
    }

    const double airspeed = air_velocity_world.Length();
    if (airspeed < 0.5) {
        return;
    }

    const math::Vector3d air_body = pose.Rot().Inverse().RotateVector(air_velocity_world);
    const double alpha_rad = std::atan2(-air_body.Z(), air_body.X());
    const double beta_rad = std::atan2(air_body.Y(), air_body.X());
    const double alpha_deg = alpha_rad * kRadToDeg;
    const double beta_deg_signed = beta_rad * kRadToDeg;
    const double beta_abs_deg = std::abs(beta_deg_signed);
    const double beta_sign = beta_deg_signed < 0.0 ? -1.0 : 1.0;

    const math::Vector3d omega_body = pose.Rot().Inverse().RotateVector(world_ang_vel->Data());
    const double p = omega_body.X();
    const double q = -omega_body.Y();
    const double r = -omega_body.Z();

    const double qbar = 0.5 * _rho * airspeed * airspeed;
    double CL = _cl_table.Interpolate(alpha_deg, beta_abs_deg);
    double CD = _cd_table.Interpolate(alpha_deg, beta_abs_deg);
    double CY = beta_sign * _cy_table.Interpolate(alpha_deg, beta_abs_deg);
    double Cm = _cm_table.Interpolate(alpha_deg, beta_abs_deg);
    double Cl = beta_sign * _cl_roll_table.Interpolate(alpha_deg, beta_abs_deg);
    double Cn = beta_sign * _cn_table.Interpolate(alpha_deg, beta_abs_deg);

    std::vector<double> control_angles_deg;
    control_angles_deg.reserve(_control_surfaces.size());

    for (const auto &surface : _control_surfaces) {
        const double angle_deg = GetControlAngleDeg(surface, _ecm) * surface.direction;
        control_angles_deg.push_back(angle_deg);

        if (surface.effect_type == "alpha_delta") {
            // Word tables for canard / elevator: coefficient increment as a
            // function of alpha and surface deflection angle. Values are total
            // aircraft increments; paired surfaces use scale=0.5 each.
            if (surface.has_cl_table) { CL += surface.scale * surface.cl_table.Interpolate(alpha_deg, angle_deg); }
            if (surface.has_cd_table) { CD += surface.scale * surface.cd_table.Interpolate(alpha_deg, angle_deg); }
            if (surface.has_cy_table) { CY += surface.scale * surface.cy_table.Interpolate(alpha_deg, angle_deg); }
            if (surface.has_cl_roll_table) { Cl += surface.scale * surface.cl_roll_table.Interpolate(alpha_deg, angle_deg); }
            if (surface.has_cm_table) { Cm += surface.scale * surface.cm_table.Interpolate(alpha_deg, angle_deg); }
            if (surface.has_cn_table) { Cn += surface.scale * surface.cn_table.Interpolate(alpha_deg, angle_deg); }
        } else if (surface.effect_type == "rudder_alpha_beta") {
            // Word rudder table is given for delta_r=+10deg versus alpha/beta.
            // Scale linearly by actual rudder deflection; table beta is signed.
            const double gain = angle_deg / std::max(std::abs(surface.ref_deflection_deg), kEpsilon);
            if (surface.has_cl_roll_table) { Cl += surface.scale * gain * surface.cl_roll_table.Interpolate(alpha_deg, beta_deg_signed); }
            if (surface.has_cd_table) { CD += surface.scale * gain * surface.cd_table.Interpolate(alpha_deg, beta_deg_signed); }
            if (surface.has_cy_table) { CY += surface.scale * gain * surface.cy_table.Interpolate(alpha_deg, beta_deg_signed); }
            if (surface.has_cn_table) { Cn += surface.scale * gain * surface.cn_table.Interpolate(alpha_deg, beta_deg_signed); }
        } else {
            // Backward-compatible legacy linear derivatives, coefficient per deg.
            CL += angle_deg * surface.cl_force;
            CD += angle_deg * surface.cd;
            CY += angle_deg * surface.cy;
            Cl += angle_deg * surface.cl_roll;
            Cm += angle_deg * surface.cm;
            Cn += angle_deg * surface.cn;
        }
    }

    const double inv_2v = 1.0 / std::max(2.0 * airspeed, kEpsilon);
    CL += _CLq * q * _mac * inv_2v;
    CY += _CYp * p * _span * inv_2v + _CYr * r * _span * inv_2v;
    Cm += _Cmq * q * _mac * inv_2v + _Cma * alpha_rad * _mac * inv_2v;
    Cl += _Clp * p * _span * inv_2v + _Clr * r * _span * inv_2v;
    Cn += _Cnp * p * _span * inv_2v + _Cnr * r * _span * inv_2v + _Cnb * beta_rad * _span * inv_2v;

    // ─── Wind-axes → body/world force conversion ───
    // Word doc: CL, CD, CY are in wind axes. In this SDF model, <upward>
    // points upward, so positive CL must generate an upward force at level
    // flight. AddWorldWrench expects a world-frame force, therefore forces are
    // first assembled in body axes and then rotated to world axes.
    const math::Vector3d wind_x_body = air_body.Normalized(); // aircraft velocity through air

    math::Vector3d lift_axis_body = _upward - wind_x_body.Dot(_upward) * wind_x_body;
    if (lift_axis_body.Length() < 1e-9) {
        lift_axis_body = _upward;
    }
    lift_axis_body.Normalize();

    // Keep the previous lateral-force sign convention while using the corrected
    // upward lift axis. At level flight this reduces to +body-Y for positive CY.
    math::Vector3d side_axis_body = lift_axis_body.Cross(wind_x_body);
    if (side_axis_body.Length() < 1e-9) {
        side_axis_body = _upward.Cross(_forward);
    }
    side_axis_body.Normalize();

    const math::Vector3d lift_body = CL * qbar * _area * lift_axis_body;
    const math::Vector3d drag_body = -CD * qbar * _area * wind_x_body;
    const math::Vector3d side_body = CY * qbar * _area * side_axis_body;
    const math::Vector3d force_body = lift_body + drag_body + side_body;
    math::Vector3d force_world = pose.Rot().RotateVector(force_body);

    // Moments: Cm, Cl, Cn are already in body axes per Word doc convention.
    // Roll  about body X, Pitch about body Y, Yaw about body Z.
    // Body axes for moments use the SDF <forward>/<upward> convention.
    const math::Vector3d body_x_axis = pose.Rot().RotateVector(_forward);
    const math::Vector3d body_z_axis = -1.0 * pose.Rot().RotateVector(_upward);
    const math::Vector3d body_y_axis = body_z_axis.Cross(body_x_axis).Normalized();

    const math::Vector3d roll_moment = Cl * qbar * _area * _span * body_x_axis;
    const math::Vector3d pitch_moment = Cm * qbar * _area * _mac * body_y_axis;
    const math::Vector3d yaw_moment = Cn * qbar * _area * _span * body_z_axis;
    const math::Vector3d torque = roll_moment + pitch_moment + yaw_moment;

    const math::Vector3d torque_body = pose.Rot().Inverse().RotateVector(torque);

    // ─── Apply force and moment at the aerodynamic reference link ───
    // Word doc: aerodynamic moment reference point == CG (-1.57, 0, 0).
    // The SDF <ac> link is placed at CG, and <cp>0 0 0</cp> means that the
    // plugin computes velocity / applies forces directly at the CG.
    // Therefore there is NO additional cp_world.Cross(force) moment arm —
    // the table moments (Cm, Cl, Cn) already account for the full moment
    // about the CG, and forces are applied at CG without extra leverage.
    force_world.Correct();
    Link link(_link_entity);
    link.AddWorldWrench(_ecm, force_world, torque);

    const double sim_time_s = std::chrono::duration<double>(_info.simTime).count();

    if (_force_log && _force_log_rate_hz > 0.0 && sim_time_s >= _next_force_log_time_s) {
        _next_force_log_time_s = sim_time_s + 1.0 / _force_log_rate_hz;

        if (!_force_log_stream.is_open()) {
            std::error_code ec;
            std::filesystem::create_directories(std::filesystem::path(_force_log_path).parent_path(), ec);
            if (ec) {
                gzwarn << "HonghuAeroTable: failed to create force log directory for "
                       << _force_log_path << ": " << ec.message() << "\n";
            }

            _force_log_stream.open(_force_log_path, std::ios::out | std::ios::trunc);
            if (_force_log_stream.is_open()) {
                _force_log_stream
                    << "time_s,airspeed_m_s,alpha_deg,beta_deg,p_rad_s,q_rad_s,r_rad_s,qbar_pa,"
                    << "CL,CD,CY,Cl,Cm,Cn,"
                    << "force_body_x_N,force_body_y_N,force_body_z_N,"
                    << "moment_body_x_Nm,moment_body_y_Nm,moment_body_z_Nm,"
                    << "force_world_x_N,force_world_y_N,force_world_z_N,"
                    << "moment_world_x_Nm,moment_world_y_Nm,moment_world_z_Nm";

                for (size_t i = 0; i < control_angles_deg.size(); ++i) {
                    _force_log_stream << ",ctrl" << i << "_deg";
                }

                _force_log_stream << "\n";
                _force_log_stream << std::setprecision(10);
                gzmsg << "HonghuAeroTable: writing aero force log to " << _force_log_path << "\n";
            } else {
                gzwarn << "HonghuAeroTable: failed to open force log " << _force_log_path << "\n";
                _force_log = false;
            }
        }

        if (_force_log_stream.is_open()) {
            _force_log_stream
                << sim_time_s << ',' << airspeed << ',' << alpha_deg << ',' << beta_deg_signed << ','
                << p << ',' << q << ',' << r << ',' << qbar << ','
                << CL << ',' << CD << ',' << CY << ',' << Cl << ',' << Cm << ',' << Cn << ','
                << force_body.X() << ',' << force_body.Y() << ',' << force_body.Z() << ','
                << torque_body.X() << ',' << torque_body.Y() << ',' << torque_body.Z() << ','
                << force_world.X() << ',' << force_world.Y() << ',' << force_world.Z() << ','
                << torque.X() << ',' << torque.Y() << ',' << torque.Z();

            for (const double angle_deg : control_angles_deg) {
                _force_log_stream << ',' << angle_deg;
            }

            _force_log_stream << "\n";
            _force_log_stream.flush();
        }
    }

    if (_debug && _debug_rate_hz > 0.0) {
        if (sim_time_s >= _next_debug_time_s) {
            _next_debug_time_s = sim_time_s + 1.0 / _debug_rate_hz;
            gzmsg << "HonghuAeroTable t=" << sim_time_s
                  << " V=" << airspeed
                  << " alpha_deg=" << alpha_deg
                  << " beta_deg=" << beta_deg_signed
                  << " CL=" << CL << " CD=" << CD << " CY=" << CY
                  << " Cm=" << Cm << " Cl=" << Cl << " Cn=" << Cn
                  << " force_body=" << force_body
                  << " force_world=" << force_world
                  << " torque=" << torque << "\n";
        }
    }
}

HonghuAeroTable::HonghuAeroTable() : _data(std::make_unique<HonghuAeroTablePrivate>()) {}
HonghuAeroTable::~HonghuAeroTable() = default;

void HonghuAeroTable::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm,
                                EventManager &)
{
    _data->Load(_entity, _sdf, _ecm);
}

void HonghuAeroTable::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
    GZ_PROFILE("HonghuAeroTable::PreUpdate");
    _data->Update(_info, _ecm);
}

} // namespace custom

GZ_ADD_PLUGIN(custom::HonghuAeroTable,
              gz::sim::System,
              custom::HonghuAeroTable::ISystemConfigure,
              custom::HonghuAeroTable::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(custom::HonghuAeroTable, "custom::HonghuAeroTable")
