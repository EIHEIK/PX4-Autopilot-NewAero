#include "HonghuPropulsionV8.hpp"
#include "HonghuV8Common.hpp"

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>

using namespace gz;
using namespace gz::sim;

namespace honghu::v8
{
namespace
{
constexpr uint32_t kInputClamped=1u<<0;
constexpr uint32_t kFuelClamped=1u<<1;
constexpr uint32_t kCommandMissing=1u<<2;
math::Vector3d FrdToGz(const math::Vector3d &v){return {v.X(),-v.Y(),-v.Z()};}
}

class HonghuPropulsionV8Private
{
public:
    bool Load(Entity entity,const std::shared_ptr<const sdf::Element> &sdf,EntityComponentManager &ecm)
    {
        _model=Model(entity);
        if(!_model.Valid(ecm)){return false;}
        const std::string link_name=sdf->Get<std::string>("link_name","base_link").first;
        const auto links=entitiesFromScopedName(link_name,ecm,_model.Entity());
        if(links.empty()){gzerr<<"HonghuPropulsionV8: link not found\n";return false;}
        _link=*links.begin();
        enableComponent<components::WorldPose>(ecm,_link);
        enableComponent<components::WorldLinearVelocity>(ecm,_link);
        _reference_altitude=sdf->Get<double>("reference_altitude_m",0.0).first;
        _tau_up=sdf->Get<double>("tau_up_s",0.5).first;
        _tau_down=sdf->Get<double>("tau_down_s",0.3).first;
        _engine_point=sdf->Get<math::Vector3d>("engine_point",{-1.23,0.0,0.12}).first;
        _propeller_rotation_sign=Clamp(sdf->Get<double>("propeller_rotation_sign",1.0).first,-1.0,1.0);
        const double down_deg=sdf->Get<double>("thrust_down_deg",3.0).first;
        _thrust_direction={std::cos(down_deg*kDegToRad),0.0,-std::sin(down_deg*kDegToRad)};
        const auto dir=ResolveDataPath(sdf->Get<std::string>("table_dir","model://honghu_wing_150kg_v8/propulsion_tables").first);
        if(!_prop.Load(dir/"propeller.csv")||!_fuel.Load(dir/"fuel.csv")){gzerr<<"HonghuPropulsionV8: table load failed from "<<dir<<"\n";return false;}
        const std::string model_name=_model.Name(ecm);
        // V8 uses the model-scoped mirror published by GZMixingInterfaceESC.
        // The bridge also retains its legacy root topic for standard models.
        _command_topic=sdf->Get<std::string>("command_topic","/model/"+model_name+"/honghu_v8/motor_command").first;
        if(!_node.Subscribe(_command_topic,&HonghuPropulsionV8Private::OnCommand,this)){gzerr<<"HonghuPropulsionV8: subscribe failed "<<_command_topic<<"\n";return false;}
        const std::string topic="/model/"+model_name+"/honghu_v8";
        _diag_pub=_node.Advertise<msgs::Double_V>(topic+"/propulsion_state");
        _speed_pub=_node.Advertise<msgs::Double>("/model/"+model_name+"/propeller_speed");
        gzmsg<<"HonghuPropulsionV8: tables loaded from "<<dir<<" command="<<_command_topic<<"\n";
        _valid=true;
        return true;
    }

    void OnCommand(const msgs::Actuators &message)
    {
        if(message.velocity_size()>0){
            _target.store(Clamp(message.velocity(0)/1000.0,0.0,1.0));
            _command_received.store(true);
        }
    }

    void Update(const UpdateInfo &info,EntityComponentManager &ecm)
    {
        if(!_valid||info.paused){return;}
        const auto pose=ecm.Component<components::WorldPose>(_link);
        const auto velocity=ecm.Component<components::WorldLinearVelocity>(_link);
        if(!pose||!velocity){return;}
        const double dt=std::chrono::duration<double>(info.dt).count();
        const double now=std::chrono::duration<double>(info.simTime).count();
        if(!_command_received.load() && now>=_next_subscription_refresh){
            _next_subscription_refresh=now+1.0;
            _node.Unsubscribe(_command_topic);
            if(!_node.Subscribe(_command_topic,&HonghuPropulsionV8Private::OnCommand,this)){
                gzerr<<"HonghuPropulsionV8: subscription refresh failed "<<_command_topic<<"\n";
            }
        }
        const double target=_target.load();
        const double tau=target>_state?_tau_up:_tau_down;
        _state+=Clamp(dt/std::max(tau,1e-3),0.0,1.0)*(target-_state);

        math::Vector3d relative_world=velocity->Data();
        const Entity wind_entity=ecm.EntityByComponents(components::Wind());
        if(wind_entity!=kNullEntity){const auto wind=ecm.Component<components::WorldLinearVelocity>(wind_entity);if(wind){relative_world-=wind->Data();}}
        const math::Vector3d velocity_gz=pose->Data().Rot().Inverse().RotateVector(relative_world);
        const math::Vector3d velocity_frd=FrdToGz(velocity_gz);
        const double airspeed=velocity_frd.Length();
        const double altitude=_reference_altitude+pose->Data().Pos().Z();
        const auto sample=_prop.Interpolate(altitude,_state,airspeed);
        bool fuel_clamped=false;
        const double fuel=_fuel.Interpolate(altitude,_state,airspeed,fuel_clamped);
        uint32_t flags=(sample.clamped?kInputClamped:0u)|(fuel_clamped?kFuelClamped:0u)|(!_command_received.load()?kCommandMissing:0u);

        const math::Vector3d force_gz=sample.thrust_newton*_thrust_direction;
        // The table reports shaft / propeller torque. The airframe receives
        // the equal and opposite reaction about the configured propeller axis.
        const math::Vector3d reaction_gz{-_propeller_rotation_sign*sample.torque_nm,0.0,0.0};
        const math::Vector3d moment_gz=_engine_point.Cross(force_gz)+reaction_gz;
        const math::Vector3d force_world=pose->Data().Rot().RotateVector(force_gz);
        const math::Vector3d moment_world=pose->Data().Rot().RotateVector(moment_gz);
        Link(_link).AddWorldWrench(ecm,force_world,moment_world);

        msgs::Double speed_message; speed_message.set_data(sample.rpm*2.0*M_PI/60.0); _speed_pub.Publish(speed_message);
        if(now>=_next_publish){
            _next_publish=now+0.01;
            msgs::Double_V state;
            for(double value:{target,_state,altitude,airspeed,sample.rpm,sample.thrust_newton,sample.torque_nm,fuel,static_cast<double>(flags)}){state.add_data(value);}
            _diag_pub.Publish(state);
        }
    }

private:
    Model _model{kNullEntity};Entity _link{kNullEntity};bool _valid{false};
    std::atomic<double> _target{0.0};std::atomic<bool> _command_received{false};
    double _state{0.0},_tau_up{0.5},_tau_down{0.3},_reference_altitude{0.0},_next_publish{0.0},_next_subscription_refresh{0.0};
    std::string _command_topic;
    math::Vector3d _engine_point{-1.23,0.0,0.12},_thrust_direction{1.0,0.0,0.0};
    double _propeller_rotation_sign{1.0};
    PropulsionTable _prop;FuelTable _fuel;transport::Node _node;transport::Node::Publisher _diag_pub,_speed_pub;
};

HonghuPropulsionV8::HonghuPropulsionV8():_data(std::make_unique<HonghuPropulsionV8Private>()){}
HonghuPropulsionV8::~HonghuPropulsionV8()=default;
void HonghuPropulsionV8::Configure(const Entity &e,const std::shared_ptr<const sdf::Element> &s,EntityComponentManager &m,EventManager &){_data->Load(e,s,m);}
void HonghuPropulsionV8::PreUpdate(const UpdateInfo &i,EntityComponentManager &m){_data->Update(i,m);}
} // namespace honghu::v8

GZ_ADD_PLUGIN(honghu::v8::HonghuPropulsionV8,gz::sim::System,
              honghu::v8::HonghuPropulsionV8::ISystemConfigure,
              honghu::v8::HonghuPropulsionV8::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(honghu::v8::HonghuPropulsionV8,"honghu::v8::HonghuPropulsionV8")
