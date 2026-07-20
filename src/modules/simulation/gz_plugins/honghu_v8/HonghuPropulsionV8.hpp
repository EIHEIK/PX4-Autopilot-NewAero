#pragma once

#include <gz/sim/System.hh>
#include <memory>

namespace honghu::v8
{
class HonghuPropulsionV8Private;
class HonghuPropulsionV8 final : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPreUpdate
{
public:
    HonghuPropulsionV8();
    ~HonghuPropulsionV8() override;
    void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;
    void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override;
private:
    std::unique_ptr<HonghuPropulsionV8Private> _data;
};
} // namespace honghu::v8
