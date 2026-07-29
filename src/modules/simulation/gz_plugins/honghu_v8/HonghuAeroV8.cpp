#include "HonghuAeroV8.hpp"
#include "HonghuV8Common.hpp"

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

using namespace gz;
using namespace gz::sim;

namespace honghu::v8
{
namespace
{
constexpr uint32_t kBetaClamped = 1u << 0;
constexpr uint32_t kPostStall = 1u << 1;
constexpr uint32_t kControlExtrapolated = 1u << 2;
constexpr uint32_t kLowSpeed = 1u << 3;
constexpr uint32_t kControlSourceClamped = 1u << 4;
constexpr uint32_t kDerivedStaticData = 1u << 5;

double SmoothStep(double low, double high, double value)
{
    const double x = Clamp((value - low) / (high - low), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

math::Vector3d FrdToGz(const math::Vector3d &v) { return {v.X(), -v.Y(), -v.Z()}; }
math::Vector3d GzToFrd(const math::Vector3d &v) { return {v.X(), -v.Y(), -v.Z()}; }

double IsaDensity(double altitude_m)
{
    const double h = Clamp(altitude_m, -500.0, 11000.0);
    const double ratio = std::max(0.1, 1.0 - 2.25577e-5 * h);
    return 1.225 * std::pow(ratio, 4.25588);
}

void SetVectorMessage(msgs::Vector3d &message, const math::Vector3d &value)
{
    message.set_x(value.X()); message.set_y(value.Y()); message.set_z(value.Z());
}
} // namespace

class HonghuAeroV8Private
{
public:
    bool Load(Entity entity, const std::shared_ptr<const sdf::Element> &sdf, EntityComponentManager &ecm)
    {
        _model = Model(entity);
        if (!_model.Valid(ecm)) { return false; }
        _link_name = sdf->Get<std::string>("link_name", "base_link").first;
        const auto links = entitiesFromScopedName(_link_name, ecm, _model.Entity());
        if (links.empty()) { gzerr << "HonghuAeroV8: link not found: " << _link_name << "\n"; return false; }
        _link = *links.begin();
        enableComponent<components::WorldPose>(ecm, _link);
        enableComponent<components::WorldLinearVelocity>(ecm, _link);
        enableComponent<components::WorldAngularVelocity>(ecm, _link);

        const std::array<std::string, 8> defaults{{"servo_0","servo_1","servo_2","servo_3","servo_4","servo_5","servo_6","servo_7"}};
        for (size_t i=0; i<defaults.size(); ++i) {
            const std::string tag = "joint_" + std::to_string(i);
            const std::string name = sdf->Get<std::string>(tag, defaults[i]).first;
            const auto entities = entitiesFromScopedName(name, ecm, _model.Entity());
            if (entities.empty()) { gzerr << "HonghuAeroV8: joint not found: " << name << "\n"; return false; }
            _joints[i] = *entities.begin();
            enableComponent<components::JointPosition>(ecm, _joints[i]);
            const std::string axis_tag = "axis_" + std::to_string(i) + "_base";
            _joint_axes[i] = sdf->Get<math::Vector3d>(axis_tag, math::Vector3d::Zero).first;
        }

        _area=sdf->Get<double>("area",2.42).first;
        _span=sdf->Get<double>("span",3.96).first;
        _mac=sdf->Get<double>("mac",0.62).first;
        _reference_altitude=sdf->Get<double>("reference_altitude_m",0.0).first;
        const auto dir=ResolveDataPath(sdf->Get<std::string>("table_dir","model://honghu_wing_150kg_v8/aero_tables").first);
        if (!LoadTables(dir)) { gzerr << "HonghuAeroV8: failed loading tables from " << dir << "\n"; return false; }

        const std::string topic = "/model/" + _model.Name(ecm) + "/honghu_v8";
        _diag_pub=_node.Advertise<msgs::Double_V>(topic+"/aero_state");
        _force_pub=_node.Advertise<msgs::Vector3d>(topic+"/force_frd");
        _moment_pub=_node.Advertise<msgs::Vector3d>(topic+"/moment_frd");
        _force_gz_pub=_node.Advertise<msgs::Vector3d>(topic+"/force_gz_flu");
        _moment_gz_pub=_node.Advertise<msgs::Vector3d>(topic+"/moment_gz_flu");

        gzmsg << "HonghuAeroV8: PDF/FRD tables loaded from " << dir << "\n";
        _valid=true;
        return true;
    }

    void Update(const UpdateInfo &info, EntityComponentManager &ecm)
    {
        if (!_valid || info.paused) { return; }

        const auto pose=ecm.Component<components::WorldPose>(_link);
        const auto velocity=ecm.Component<components::WorldLinearVelocity>(_link);
        const auto angular=ecm.Component<components::WorldAngularVelocity>(_link);
        if (!pose || !velocity || !angular) { return; }

        math::Vector3d relative_world=velocity->Data();
        const Entity wind_entity=ecm.EntityByComponents(components::Wind());
        if (wind_entity != kNullEntity) {
            const auto wind=ecm.Component<components::WorldLinearVelocity>(wind_entity);
            if (wind) { relative_world -= wind->Data(); }
        }
        const math::Vector3d velocity_gz=pose->Data().Rot().Inverse().RotateVector(relative_world);
        const math::Vector3d velocity_frd=GzToFrd(velocity_gz);
        const double speed=velocity_frd.Length();
        if (speed < 0.05) { ResetAngles(); }

        const double alpha=speed<0.05?0.0:std::atan2(velocity_frd.Z(),velocity_frd.X());
        const double beta=speed<0.05?0.0:std::atan2(velocity_frd.Y(),std::hypot(velocity_frd.X(),velocity_frd.Z()));
        const double alpha_deg=alpha*kRadToDeg;
        const double beta_deg=beta*kRadToDeg;
        uint32_t flags=0;
        if (std::abs(beta_deg)>16.0) { flags|=kBetaClamped; }
        if (alpha_deg>20.0 || alpha_deg<-12.0) { flags|=kPostStall; }
        // The PDF supplies static alpha rows from -2 to 20 deg.  The -12 deg
        // negative anchor and alpha=18/20 lateral / nonzero-beta shapes are
        // explicitly derived data, so expose their use to every test client.
        if (alpha_deg<-2.0 || alpha_deg>16.0) { flags|=kDerivedStaticData; }

        const double dt=std::chrono::duration<double>(info.dt).count();
        UpdateAngleRates(alpha,beta,speed,dt,flags);
        const auto omega_gz=pose->Data().Rot().Inverse().RotateVector(angular->Data());
        const math::Vector3d omega_frd=GzToFrd(omega_gz);
        Coefficients coefficients=StaticCoefficients(alpha_deg,beta_deg);

        const auto theta=JointAngles(ecm);
        const double delta_a=0.5*(-theta[0]+theta[1]);
        const double delta_e=0.5*(theta[2]+theta[3]);
        const double delta_r=0.5*(theta[4]+theta[5]);
        const double delta_c=0.5*(theta[6]+theta[7]);
        std::array<Coefficients,4> control_contributions{};
        AddControls(alpha_deg,beta_deg,delta_a,delta_e,delta_r,delta_c,coefficients,control_contributions,flags);

        const double rate_blend=SmoothStep(3.0,5.0,speed);
        const double inv2v=0.5/std::max(speed,5.0);
        coefficients.CL += rate_blend * 5.62 * omega_frd.Y()*_mac*inv2v;
        coefficients.CY += rate_blend * (-0.15*omega_frd.X()*_span*inv2v + 0.34*omega_frd.Z()*_span*inv2v);
        coefficients.Cl += rate_blend * (-0.33*omega_frd.X()*_span*inv2v + 0.10*omega_frd.Z()*_span*inv2v);
        coefficients.Cm += rate_blend * (-7.0*omega_frd.Y()*_mac*inv2v - 0.33*_alpha_dot*_mac*inv2v);
        coefficients.Cn += rate_blend * (-0.05*omega_frd.X()*_span*inv2v - 0.08*omega_frd.Z()*_span*inv2v
                                          + 0.14*_beta_dot*_span*inv2v);

        const double rho=IsaDensity(_reference_altitude+pose->Data().Pos().Z());
        const double qbar=0.5*rho*speed*speed;
        const double ca=std::cos(alpha), sa=std::sin(alpha), cb=std::cos(beta), sb=std::sin(beta);
        const math::Vector3d ex{ca*cb,sb,sa*cb};
        const math::Vector3d ey{-ca*sb,cb,-sa*sb};
        const math::Vector3d ez{-sa,0.0,ca};
        const math::Vector3d force_frd=(-coefficients.CD*qbar*_area)*ex
                                      +(coefficients.CY*qbar*_area)*ey
                                      +(-coefficients.CL*qbar*_area)*ez;
        const math::Vector3d moment_frd{coefficients.Cl*qbar*_area*_span,
                                        coefficients.Cm*qbar*_area*_mac,
                                        coefficients.Cn*qbar*_area*_span};
        const math::Vector3d force_gz=FrdToGz(force_frd);
        const math::Vector3d moment_gz=FrdToGz(moment_frd);
        const math::Vector3d force_world=pose->Data().Rot().RotateVector(force_gz);
        const math::Vector3d moment_world=pose->Data().Rot().RotateVector(moment_gz);
        Link(_link).AddWorldWrench(ecm,force_world,moment_world);
        Publish(speed,alpha_deg,beta_deg,rho,omega_frd,coefficients,theta,{delta_a,delta_e,delta_r,delta_c},
                control_contributions,force_frd,moment_frd,force_gz,moment_gz,flags,info);
    }

private:
    bool LoadTables(const std::filesystem::path &dir)
    {
        bool ok=true;
        ok &= _cl.Load(dir/"CL.csv"); ok &= _cd.Load(dir/"CD.csv"); ok &= _cy.Load(dir/"CY.csv");
        ok &= _roll.Load(dir/"Cl.csv"); ok &= _pitch.Load(dir/"Cm.csv"); ok &= _yaw.Load(dir/"Cn.csv");
        const auto c=dir/"control_tables";
        ok &= _canard_cl.Load(c/"canard_CL.csv"); ok &= _canard_cd.Load(c/"canard_CD.csv"); ok &= _canard_cm.Load(c/"canard_Cm.csv");
        ok &= _elevator_cl.Load(c/"elevator_CL.csv"); ok &= _elevator_cd.Load(c/"elevator_CD.csv"); ok &= _elevator_cm.Load(c/"elevator_Cm.csv");
        ok &= _aileron_cd.Load(c/"aileron_CD.csv"); ok &= _aileron_cy.Load(c/"aileron_CY.csv"); ok &= _aileron_cl.Load(c/"aileron_Cl.csv"); ok &= _aileron_cn.Load(c/"aileron_Cn.csv");
        ok &= _rudder_cd.Load(c/"rudder_CD.csv"); ok &= _rudder_cy.Load(c/"rudder_CY.csv"); ok &= _rudder_cl.Load(c/"rudder_Cl.csv"); ok &= _rudder_cn.Load(c/"rudder_Cn.csv");
        return ok;
    }

    double StaticValue(const Grid2D &table,double alpha,double beta) const
    {
        if (alpha>=table.RowMin()) { return table.Interpolate(alpha,std::abs(beta)); }
        const double v0=table.Interpolate(table.RowMin(),std::abs(beta));
        const double v1=table.Interpolate(table.RowMin()+2.0,std::abs(beta));
        return v0+(alpha-table.RowMin())*(v1-v0)/2.0;
    }

    double Viterna(const Grid2D &lift,const Grid2D &drag,double alpha,double beta,bool want_lift) const
    {
        const double anchor=alpha>0.0?20.0:-12.0;
        const double a0=anchor*kDegToRad, a=Clamp(alpha,-89.0,89.0)*kDegToRad;
        const double cls=StaticValue(lift,anchor,beta), cds=StaticValue(drag,anchor,beta);
        const double cdmax=1.11+0.018*(_span*_span/_area);
        const double b2=(cds-cdmax*std::sin(a0)*std::sin(a0))/std::cos(a0);
        const double a1=0.5*cdmax;
        const double a2=(cls-cdmax*std::sin(a0)*std::cos(a0))*std::sin(a0)/(std::cos(a0)*std::cos(a0));
        if (want_lift) { return a1*std::sin(2.0*a)+a2*std::cos(a)*std::cos(a)/std::sin(a); }
        return cdmax*std::sin(a)*std::sin(a)+b2*std::cos(a);
    }

    Coefficients StaticCoefficients(double alpha,double beta) const
    {
        const double abs_beta=Clamp(std::abs(beta),0.0,16.0);
        const double sign=beta<0.0?-1.0:1.0;
        Coefficients c;
        if (alpha>20.0 || alpha<-12.0) {
            c.CL=Viterna(_cl,_cd,alpha,abs_beta,true);
            c.CD=std::max(0.0,Viterna(_cl,_cd,alpha,abs_beta,false));
            const double anchor=alpha>0.0?20.0:-12.0;
            const double fade=1.0-SmoothStep(std::abs(anchor),90.0,std::abs(alpha));
            c.Cm=StaticValue(_pitch,anchor,abs_beta)*fade;
        } else {
            c.CL=StaticValue(_cl,alpha,abs_beta); c.CD=std::max(0.0,StaticValue(_cd,alpha,abs_beta)); c.Cm=StaticValue(_pitch,alpha,abs_beta);
        }
        const double lateral_fade=1.0-SmoothStep(16.0,90.0,std::abs(alpha));
        c.CY=sign*StaticValue(_cy,alpha,abs_beta)*lateral_fade;
        c.Cl=sign*StaticValue(_roll,alpha,abs_beta)*lateral_fade;
        c.Cn=sign*StaticValue(_yaw,alpha,abs_beta)*lateral_fade;
        return c;
    }

    void AddControls(double alpha,double beta,double da,double de,double dr,double dc,Coefficients &c,
                     std::array<Coefficients,4> &contribution,uint32_t &flags) const
    {
        if (std::abs(da)>10.0 || de<-10.0 || de>20.0 || std::abs(dr)>10.0 || dc<-4.0 || dc>8.0) {
            flags|=kControlExtrapolated;
        }
        // Grid2D deliberately clamps at source-table boundaries.  Distinguish
        // that bounded behavior from measured data so post-flight analysis
        // never mistakes a numerically safe value for a validated coefficient.
        const bool aileron_source_clamped=std::abs(da)>1e-9 && (alpha<0.0 || alpha>12.0);
        const bool elevator_source_clamped=std::abs(de)>1e-9 && (alpha<0.0 || alpha>12.0);
        const bool rudder_source_clamped=std::abs(dr)>1e-9
                                         && (alpha<0.0 || alpha>8.0 || std::abs(beta)>12.0);
        const bool canard_source_clamped=std::abs(dc)>1e-9 && (alpha<0.0 || alpha>12.0);
        if (aileron_source_clamped || elevator_source_clamped || rudder_source_clamped || canard_source_clamped) {
            flags|=kControlSourceClamped;
        }

        auto &aileron=contribution[0];
        const double aileron_fade=1.0-SmoothStep(12.0,20.0,std::abs(alpha));
        const double da_lookup=Clamp(da,-10.0,10.0);
        aileron.CD=_aileron_cd.Interpolate(alpha,da_lookup)*da*aileron_fade;
        aileron.CY=_aileron_cy.Interpolate(alpha,da_lookup)*da*aileron_fade;
        aileron.Cl=_aileron_cl.Interpolate(alpha,da_lookup)*da*aileron_fade;
        aileron.Cn=_aileron_cn.Interpolate(alpha,da_lookup)*da*aileron_fade;

        auto &elevator=contribution[1];
        const double de_lookup=Clamp(de,-10.0,20.0);
        elevator.CL=_elevator_cl.Interpolate(alpha,de_lookup)*de;
        elevator.CD=_elevator_cd.Interpolate(alpha,de_lookup)*de;
        elevator.Cm=_elevator_cm.Interpolate(alpha,de_lookup)*de;

        auto &rudder=contribution[2];
        const double reflected_beta=dr<0.0?-beta:beta;
        rudder.CD=(dr<0.0?-1.0:1.0)*_rudder_cd.Interpolate(alpha,reflected_beta)*dr;
        rudder.CY=_rudder_cy.Interpolate(alpha,reflected_beta)*dr;
        rudder.Cl=_rudder_cl.Interpolate(alpha,reflected_beta)*dr;
        rudder.Cn=_rudder_cn.Interpolate(alpha,reflected_beta)*dr;

        auto &canard=contribution[3];
        const double dc_effective=dc<-4.0?-4.0:Clamp(dc,-4.0,15.0);
        const double dc_lookup=Clamp(dc_effective,-4.0,8.0);
        const double canard_fade=1.0-SmoothStep(12.0,16.0,std::abs(alpha+dc_effective));
        canard.CL=_canard_cl.Interpolate(alpha,dc_lookup)*dc_effective*canard_fade;
        canard.CD=_canard_cd.Interpolate(alpha,dc_lookup)*dc_effective*canard_fade;
        canard.Cm=_canard_cm.Interpolate(alpha,dc_lookup)*dc_effective*canard_fade;

        for (const auto &delta : contribution) {
            c.CL+=delta.CL; c.CD+=delta.CD; c.CY+=delta.CY;
            c.Cl+=delta.Cl; c.Cm+=delta.Cm; c.Cn+=delta.Cn;
        }
    }

    std::array<double,8> JointAngles(EntityComponentManager &ecm) const
    {
        std::array<double,8> result{};
        for (size_t i=0;i<result.size();++i) {
            const auto joint=ecm.Component<components::JointPosition>(_joints[i]);
            if (joint && !joint->Data().empty()) { result[i]=joint->Data()[0]*kRadToDeg; }
        }
        return result;
    }

    void UpdateAngleRates(double alpha,double beta,double speed,double dt,uint32_t &flags)
    {
        if (speed<3.0 || dt<=0.0 || !_have_angles) { _previous_alpha=alpha; _previous_beta=beta; _alpha_dot=0.0; _beta_dot=0.0; _have_angles=true; flags|=kLowSpeed; return; }
        const double raw_alpha=Clamp(std::remainder(alpha-_previous_alpha,2.0*GZ_PI)/dt,-10.0,10.0);
        const double raw_beta=Clamp(std::remainder(beta-_previous_beta,2.0*GZ_PI)/dt,-10.0,10.0);
        const double gain=dt/(0.05+dt);
        _alpha_dot+=gain*(raw_alpha-_alpha_dot); _beta_dot+=gain*(raw_beta-_beta_dot);
        _previous_alpha=alpha; _previous_beta=beta;
    }
    void ResetAngles() { _have_angles=false; _alpha_dot=0.0; _beta_dot=0.0; }

    void Publish(double speed,double alpha,double beta,double rho,const math::Vector3d &omega_frd,
                 const Coefficients &c,const std::array<double,8> &theta,const std::array<double,4> &delta,
                 const std::array<Coefficients,4> &contribution,const math::Vector3d &force_frd,
                 const math::Vector3d &moment_frd,const math::Vector3d &force_gz,const math::Vector3d &moment_gz,
                 uint32_t flags,const UpdateInfo &info)
    {
        const double now=std::chrono::duration<double>(info.simTime).count();
        if (now<_next_publish) { return; }
        // Diagnostics are flight-recorder inputs, not part of the force path.
        // 50 Hz resolves the 4 rad/s control-surface motion while keeping the
        // transport and ULog load well below the former 100 Hz stream.
        _next_publish=now+0.02;
        msgs::Double_V state;
        for (double value : {speed,alpha,beta,rho,_alpha_dot,_beta_dot,omega_frd.X(),omega_frd.Y(),omega_frd.Z(),
                             c.CL,c.CD,c.CY,c.Cl,c.Cm,c.Cn}) { state.add_data(value); }
        for (double value : theta) { state.add_data(value); }
        for (double value : delta) { state.add_data(value); }
        for (const auto &item : contribution) {
            for (double value : {item.CL,item.CD,item.CY,item.Cl,item.Cm,item.Cn}) { state.add_data(value); }
        }
        for (const auto &axis : _joint_axes) {
            state.add_data(axis.X()); state.add_data(axis.Y()); state.add_data(axis.Z());
        }
        state.add_data(static_cast<double>(flags));
        // Preserve source simulation time.  The Gazebo bridge must not stamp a
        // queued sample with its later callback-arrival time.
        state.add_data(now*1e6);
        state.add_data(static_cast<double>(_diagnostic_sequence++));
        _diag_pub.Publish(state);

        msgs::Vector3d force_frd_msg,moment_frd_msg,force_gz_msg,moment_gz_msg;
        SetVectorMessage(force_frd_msg,force_frd); SetVectorMessage(moment_frd_msg,moment_frd);
        SetVectorMessage(force_gz_msg,force_gz); SetVectorMessage(moment_gz_msg,moment_gz);
        _force_pub.Publish(force_frd_msg); _moment_pub.Publish(moment_frd_msg);
        _force_gz_pub.Publish(force_gz_msg); _moment_gz_pub.Publish(moment_gz_msg);
    }

    Model _model{kNullEntity}; Entity _link{kNullEntity}; std::array<Entity,8> _joints{};
    std::array<math::Vector3d,8> _joint_axes{}; std::string _link_name;
    bool _valid{false},_have_angles{false}; double _area{2.42},_span{3.96},_mac{0.62},_reference_altitude{0.0};
    double _previous_alpha{0.0},_previous_beta{0.0},_alpha_dot{0.0},_beta_dot{0.0},_next_publish{0.0};
    uint32_t _diagnostic_sequence{0};
    Grid2D _cl,_cd,_cy,_roll,_pitch,_yaw;
    Grid2D _canard_cl,_canard_cd,_canard_cm,_elevator_cl,_elevator_cd,_elevator_cm;
    Grid2D _aileron_cd,_aileron_cy,_aileron_cl,_aileron_cn,_rudder_cd,_rudder_cy,_rudder_cl,_rudder_cn;
    transport::Node _node;
    transport::Node::Publisher _diag_pub,_force_pub,_moment_pub,_force_gz_pub,_moment_gz_pub;
};

HonghuAeroV8::HonghuAeroV8():_data(std::make_unique<HonghuAeroV8Private>()){}
HonghuAeroV8::~HonghuAeroV8()=default;
void HonghuAeroV8::Configure(const Entity &e,const std::shared_ptr<const sdf::Element> &s,EntityComponentManager &m,EventManager &){_data->Load(e,s,m);}
void HonghuAeroV8::PreUpdate(const UpdateInfo &i,EntityComponentManager &m){_data->Update(i,m);}
} // namespace honghu::v8

GZ_ADD_PLUGIN(honghu::v8::HonghuAeroV8, gz::sim::System,
              honghu::v8::HonghuAeroV8::ISystemConfigure,
              honghu::v8::HonghuAeroV8::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(honghu::v8::HonghuAeroV8,"honghu::v8::HonghuAeroV8")
