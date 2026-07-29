#pragma once

#include <gz/sim/System.hh>

#include <memory>

namespace honghu::v8
{
class HonghuMagnetometerV8Private;

class HonghuMagnetometerV8 final : public gz::sim::System,
                                    public gz::sim::ISystemConfigure,
                                    public gz::sim::ISystemPostUpdate
{
public:
    HonghuMagnetometerV8();
    ~HonghuMagnetometerV8() override;

    void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;
    void PostUpdate(const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &) override;

private:
    std::unique_ptr<HonghuMagnetometerV8Private> _data;
};
} // namespace honghu::v8
