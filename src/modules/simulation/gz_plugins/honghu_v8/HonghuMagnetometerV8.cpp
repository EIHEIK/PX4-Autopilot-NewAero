#include "HonghuMagnetometerV8.hpp"

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <chrono>
#include <random>
#include <string>

using namespace gz;
using namespace gz::sim;

namespace honghu::v8
{
class HonghuMagnetometerV8Private
{
public:
    bool Load(Entity entity, const std::shared_ptr<const sdf::Element> &sdf,
              EntityComponentManager &ecm)
    {
        _model = Model(entity);

        if (!_model.Valid(ecm)) {
            gzerr << "HonghuMagnetometerV8: invalid model entity\n";
            return false;
        }

        const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;
        const auto links = entitiesFromScopedName(link_name, ecm, _model.Entity());

        if (links.empty()) {
            gzerr << "HonghuMagnetometerV8: link not found: " << link_name << "\n";
            return false;
        }

        _link = *links.begin();
        enableComponent<components::WorldPose>(ecm, _link);
        _field_ned_gauss = sdf->Get<math::Vector3d>(
                                   "field_ned_gauss", {0.346940371, -0.035562102, 0.325102706}).first;
        _noise_stddev_gauss = sdf->Get<double>("noise_stddev_gauss", 0.0001).first;
        _period_s = 1.0 / sdf->Get<double>("update_rate_hz", 100.0).first;
        _noise = std::normal_distribution<double>(0.0, _noise_stddev_gauss);

        const Entity world = ecm.EntityByComponents(components::World());
        const auto world_name = world == kNullEntity ? nullptr : ecm.Component<components::Name>(world);

        if (!world_name) {
            gzerr << "HonghuMagnetometerV8: world name unavailable\n";
            return false;
        }

        const std::string model_name = _model.Name(ecm);
        _topic = "/world/" + world_name->Data() + "/model/" + model_name
                 + "/link/" + link_name + "/sensor/magnetometer_sensor/magnetometer";
        _publisher = _node.Advertise<msgs::Magnetometer>(_topic);

        if (!_publisher) {
            gzerr << "HonghuMagnetometerV8: advertise failed: " << _topic << "\n";
            return false;
        }

        gzmsg << "HonghuMagnetometerV8: exact NED field " << _field_ned_gauss
              << " gauss, topic=" << _topic << "\n";
        _valid = true;
        return true;
    }

    void Update(const UpdateInfo &info, const EntityComponentManager &ecm)
    {
        if (!_valid || info.paused) {
            return;
        }

        const double now = std::chrono::duration<double>(info.simTime).count();

        if (now + 1e-9 < _next_publish_s) {
            return;
        }

        _next_publish_s = now + _period_s;
        const auto pose = ecm.Component<components::WorldPose>(_link);

        if (!pose) {
            return;
        }

        // Earth NED -> Gazebo ENU: [N,E,D] -> [E,N,-D].
        const math::Vector3d field_enu{
            _field_ned_gauss.Y(), _field_ned_gauss.X(), -_field_ned_gauss.Z()
        };
        const math::Vector3d field_flu = pose->Data().Rot().Inverse().RotateVector(field_enu);

        // Gazebo body FLU -> PX4 body FRD.
        math::Vector3d field_frd{field_flu.X(), -field_flu.Y(), -field_flu.Z()};
        field_frd += math::Vector3d{_noise(_rng), _noise(_rng), _noise(_rng)};

        // Gazebo Harmonic's PX4 bridge applies [-Y,-X,+Z]. Publish the exact
        // inverse legacy representation so the unchanged official callback
        // produces field_frd. This adapter is V8-only and must not be used with
        // the future standard ENU/Tesla bridge path.
        msgs::Magnetometer message;
        *message.mutable_header()->mutable_stamp() = msgs::Convert(info.simTime);
        auto frame = message.mutable_header()->add_data();
        frame->set_key("frame_id");
        frame->add_value("base_link");
        message.mutable_field_tesla()->set_x(-field_frd.Y());
        message.mutable_field_tesla()->set_y(-field_frd.X());
        message.mutable_field_tesla()->set_z(field_frd.Z());
        _publisher.Publish(message);
    }

private:
    Model _model{kNullEntity};
    Entity _link{kNullEntity};
    bool _valid{false};
    double _period_s{0.01};
    double _next_publish_s{0.0};
    double _noise_stddev_gauss{0.0001};
    math::Vector3d _field_ned_gauss{0.346940371, -0.035562102, 0.325102706};
    std::mt19937 _rng{0x484F4E47u};
    std::normal_distribution<double> _noise{0.0, 0.0001};
    std::string _topic;
    transport::Node _node;
    transport::Node::Publisher _publisher;
};

HonghuMagnetometerV8::HonghuMagnetometerV8()
    : _data(std::make_unique<HonghuMagnetometerV8Private>())
{
}

HonghuMagnetometerV8::~HonghuMagnetometerV8() = default;

void HonghuMagnetometerV8::Configure(const Entity &entity,
                                     const std::shared_ptr<const sdf::Element> &sdf,
                                     EntityComponentManager &ecm, EventManager &)
{
    _data->Load(entity, sdf, ecm);
}

void HonghuMagnetometerV8::PostUpdate(const UpdateInfo &info,
                                      const EntityComponentManager &ecm)
{
    _data->Update(info, ecm);
}
} // namespace honghu::v8

GZ_ADD_PLUGIN(honghu::v8::HonghuMagnetometerV8, gz::sim::System,
              honghu::v8::HonghuMagnetometerV8::ISystemConfigure,
              honghu::v8::HonghuMagnetometerV8::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(honghu::v8::HonghuMagnetometerV8,
                    "honghu::v8::HonghuMagnetometerV8")
