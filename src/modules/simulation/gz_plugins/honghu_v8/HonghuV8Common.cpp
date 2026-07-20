#include "HonghuV8Common.hpp"

#include <gz/common/Console.hh>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace honghu::v8
{
namespace
{
std::string Trim(const std::string &value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { return {}; }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> Split(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) { fields.push_back(Trim(field)); }
    return fields;
}

std::pair<size_t, size_t> Bracket(const std::vector<double> &axis, double query)
{
    auto upper = std::upper_bound(axis.begin(), axis.end(), query);
    size_t hi = static_cast<size_t>(std::distance(axis.begin(), upper));
    if (hi == 0) { hi = 1; }
    if (hi >= axis.size()) { hi = axis.size() - 1; }
    return {hi - 1, hi};
}

double Fraction(double x0, double x1, double x)
{
    return std::abs(x1 - x0) < 1e-12 ? 0.0 : (x - x0) / (x1 - x0);
}

bool Same(double a, double b)
{
    return std::abs(a - b) < 1e-9;
}

void UniqueSort(std::vector<double> &values)
{
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}
} // namespace

double Clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }
double Lerp(double a, double b, double t) { return a + (b - a) * t; }

std::filesystem::path ResolveDataPath(const std::string &raw)
{
    constexpr const char *prefix = "model://";
    const bool model_uri = raw.rfind(prefix, 0) == 0;
    const std::filesystem::path path(model_uri ? raw.substr(std::char_traits<char>::length(prefix)) : raw);
    if (!model_uri && path.is_absolute() && std::filesystem::exists(path)) { return path; }
    if (!model_uri && std::filesystem::exists(path)) { return std::filesystem::absolute(path); }
    if (const char *resource = std::getenv("GZ_SIM_RESOURCE_PATH")) {
        std::stringstream paths(resource);
        std::string base;
        while (std::getline(paths, base, ':')) {
            if (base.empty()) { continue; }
            const auto root = std::filesystem::path(base);
            auto candidate = root / path;
            if (std::filesystem::exists(candidate)) { return candidate; }
            if (root.filename() == "models" && root.parent_path().filename() == "gz"
                && root.parent_path().parent_path().filename() == "simulation"
                && root.parent_path().parent_path().parent_path().filename() == "Tools") {
                const auto repo = root.parent_path().parent_path().parent_path().parent_path();
                candidate = repo / "simulation_models" / "models" / path;
                if (std::filesystem::exists(candidate)) { return candidate; }
            }
        }
    }
    return path;
}

bool Grid2D::Load(const std::filesystem::path &path)
{
    std::ifstream file(path);
    if (!file) { gzerr << "HonghuV8: cannot open grid " << path << "\n"; return false; }
    std::string line;
    bool header_read = false;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line.front() == '#') { continue; }
        const auto fields = Split(line);
        if (!header_read) {
            for (size_t i = 1; i < fields.size(); ++i) { _columns.push_back(std::stod(fields[i])); }
            header_read = true;
            continue;
        }
        if (fields.size() != _columns.size() + 1) { return false; }
        _rows.push_back(std::stod(fields[0]));
        std::vector<double> row;
        for (size_t i = 1; i < fields.size(); ++i) { row.push_back(std::stod(fields[i])); }
        _values.push_back(std::move(row));
    }
    return _rows.size() >= 2 && _columns.size() >= 2 && _values.size() == _rows.size();
}

double Grid2D::Interpolate(double row_query, double column_query) const
{
    const double row = Clamp(row_query, _rows.front(), _rows.back());
    const double column = Clamp(column_query, _columns.front(), _columns.back());
    const auto [r0, r1] = Bracket(_rows, row);
    const auto [c0, c1] = Bracket(_columns, column);
    const double tr = Fraction(_rows[r0], _rows[r1], row);
    const double tc = Fraction(_columns[c0], _columns[c1], column);
    return Lerp(Lerp(_values[r0][c0], _values[r0][c1], tc),
                Lerp(_values[r1][c0], _values[r1][c1], tc), tr);
}

bool PropulsionTable::Load(const std::filesystem::path &path)
{
    std::ifstream file(path);
    if (!file) { return false; }
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
        if (Trim(line).empty()) { continue; }
        const auto f = Split(line);
        if (f.size() < 6) { return false; }
        Row row{std::stod(f[0]), std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]), std::stod(f[5])};
        _rows.push_back(row);
        _altitudes.push_back(row.altitude);
    }
    UniqueSort(_altitudes);
    return _altitudes.size() >= 2 && !_rows.empty();
}

PropulsionSample PropulsionTable::AtAltitude(double altitude, double throttle_pct, double airspeed) const
{
    std::vector<double> throttles{0.0};
    for (const auto &row : _rows) { if (Same(row.altitude, altitude)) { throttles.push_back(row.throttle); } }
    UniqueSort(throttles);
    const double throttle = Clamp(throttle_pct, 0.0, throttles.back());
    const auto [t0i, t1i] = Bracket(throttles, throttle);
    auto at_level = [&](double level) {
        PropulsionSample sample;
        if (Same(level, 0.0)) { return sample; }
        std::vector<Row> rows;
        for (const auto &row : _rows) {
            if (Same(row.altitude, altitude) && Same(row.throttle, level)) { rows.push_back(row); }
        }
        std::sort(rows.begin(), rows.end(), [](const Row &a, const Row &b) { return a.airspeed < b.airspeed; });
        const double v = Clamp(airspeed, rows.front().airspeed, rows.back().airspeed);
        std::vector<double> speeds;
        for (const auto &row : rows) { speeds.push_back(row.airspeed); }
        const auto [v0, v1] = Bracket(speeds, v);
        const double tv = Fraction(speeds[v0], speeds[v1], v);
        sample.rpm = rows[v0].rpm;
        sample.thrust_newton = Lerp(rows[v0].thrust_kgf, rows[v1].thrust_kgf, tv) * kKgfToNewton;
        sample.torque_nm = Lerp(rows[v0].torque, rows[v1].torque, tv);
        return sample;
    };
    const auto a = at_level(throttles[t0i]);
    const auto b = at_level(throttles[t1i]);
    const double t = Fraction(throttles[t0i], throttles[t1i], throttle);
    return {Lerp(a.rpm,b.rpm,t), Lerp(a.thrust_newton,b.thrust_newton,t), Lerp(a.torque_nm,b.torque_nm,t), false};
}

PropulsionSample PropulsionTable::Interpolate(double altitude_m, double throttle, double airspeed_mps) const
{
    const double altitude = Clamp(altitude_m, _altitudes.front(), _altitudes.back());
    const double speed = Clamp(airspeed_mps, 0.0, 50.0);
    const double command = Clamp(throttle, 0.0, 1.0);
    const auto [h0, h1] = Bracket(_altitudes, altitude);
    auto a = AtAltitude(_altitudes[h0], command * 100.0, speed);
    auto b = AtAltitude(_altitudes[h1], command * 100.0, speed);
    const double t = Fraction(_altitudes[h0], _altitudes[h1], altitude);
    PropulsionSample result{Lerp(a.rpm,b.rpm,t), Lerp(a.thrust_newton,b.thrust_newton,t), Lerp(a.torque_nm,b.torque_nm,t), false};
    result.clamped = !Same(altitude, altitude_m) || !Same(speed, airspeed_mps) || !Same(command, throttle);
    return result;
}

bool FuelTable::Load(const std::filesystem::path &path)
{
    std::ifstream file(path);
    if (!file) { return false; }
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
        const auto f = Split(line);
        if (f.size() < 4) { continue; }
        const double h=std::stod(f[0]), v=std::stod(f[1]), t=std::stod(f[2]);
        _values[{h,v,t}]=std::stod(f[3]);
        _altitudes.push_back(h); _airspeeds.push_back(v); _throttles.push_back(t);
    }
    UniqueSort(_altitudes); UniqueSort(_airspeeds); UniqueSort(_throttles);
    return _altitudes.size() >= 2 && _airspeeds.size() >= 2 && _throttles.size() >= 2;
}

double FuelTable::Interpolate(double altitude_m, double throttle, double airspeed_mps, bool &clamped) const
{
    const double h=Clamp(altitude_m,_altitudes.front(),_altitudes.back());
    const double v=Clamp(airspeed_mps,_airspeeds.front(),_airspeeds.back());
    const double t=Clamp(throttle*100.0,_throttles.front(),_throttles.back());
    clamped = !Same(h, altitude_m) || !Same(v, airspeed_mps) || !Same(t, throttle * 100.0);
    const auto [h0,h1]=Bracket(_altitudes,h); const auto [v0,v1]=Bracket(_airspeeds,v); const auto [t0,t1]=Bracket(_throttles,t);
    auto value=[&](size_t ih,size_t iv,size_t it){ return _values.at({_altitudes[ih],_airspeeds[iv],_throttles[it]}); };
    const double fh=Fraction(_altitudes[h0],_altitudes[h1],h), fv=Fraction(_airspeeds[v0],_airspeeds[v1],v), ft=Fraction(_throttles[t0],_throttles[t1],t);
    const double c00=Lerp(value(h0,v0,t0),value(h0,v0,t1),ft);
    const double c01=Lerp(value(h0,v1,t0),value(h0,v1,t1),ft);
    const double c10=Lerp(value(h1,v0,t0),value(h1,v0,t1),ft);
    const double c11=Lerp(value(h1,v1,t0),value(h1,v1,t1),ft);
    return Lerp(Lerp(c00,c01,fv),Lerp(c10,c11,fv),fh);
}
} // namespace honghu::v8
