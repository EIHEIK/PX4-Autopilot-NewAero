#pragma once

#include <gz/sim/System.hh>
#include <memory>

namespace honghu::v8
{
class HonghuAeroV8Private;
class HonghuAeroV8 final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
public:
    HonghuAeroV8();
    ~HonghuAeroV8() override;
    void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;
    void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override;
private:
    std::unique_ptr<HonghuAeroV8Private> _data;
};
} // namespace honghu::v8
