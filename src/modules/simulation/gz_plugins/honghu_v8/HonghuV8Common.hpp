#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace honghu::v8
{
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kKgfToNewton = 9.80665;

double Clamp(double value, double low, double high);
double Lerp(double a, double b, double t);
std::filesystem::path ResolveDataPath(const std::string &raw);

class Grid2D
{
public:
    bool Load(const std::filesystem::path &path);
    double Interpolate(double row_query, double column_query) const;
    double RowMin() const { return _rows.front(); }
    double RowMax() const { return _rows.back(); }
    double ColumnMin() const { return _columns.front(); }
    double ColumnMax() const { return _columns.back(); }

private:
    std::vector<double> _rows;
    std::vector<double> _columns;
    std::vector<std::vector<double>> _values;
};

struct PropulsionSample
{
    double rpm{0.0};
    double thrust_newton{0.0};
    double torque_nm{0.0};
    bool clamped{false};
};

class PropulsionTable
{
public:
    bool Load(const std::filesystem::path &path);
    PropulsionSample Interpolate(double altitude_m, double throttle, double airspeed_mps) const;

private:
    struct Row { double altitude; double throttle; double rpm; double airspeed; double thrust_kgf; double torque; };
    PropulsionSample AtAltitude(double altitude, double throttle_pct, double airspeed) const;
    std::vector<Row> _rows;
    std::vector<double> _altitudes;
};

class FuelTable
{
public:
    bool Load(const std::filesystem::path &path);
    double Interpolate(double altitude_m, double throttle, double airspeed_mps, bool &clamped) const;

private:
    std::map<std::tuple<double, double, double>, double> _values;
    std::vector<double> _altitudes;
    std::vector<double> _airspeeds;
    std::vector<double> _throttles;
};

struct Coefficients
{
    double CL{0.0};
    double CD{0.0};
    double CY{0.0};
    double Cl{0.0};
    double Cm{0.0};
    double Cn{0.0};
};
} // namespace honghu::v8
