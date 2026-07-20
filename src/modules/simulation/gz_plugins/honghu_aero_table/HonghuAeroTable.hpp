/****************************************************************************
 * Honghu lookup-table aerodynamic plugin for Gazebo SITL.
 ****************************************************************************/

#pragma once

#include <gz/sim/System.hh>

#include <memory>

namespace custom
{
class HonghuAeroTablePrivate;

class HonghuAeroTable:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    HonghuAeroTable();
    ~HonghuAeroTable() override;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override;

private:
    std::unique_ptr<HonghuAeroTablePrivate> _data;
};
} // namespace custom
