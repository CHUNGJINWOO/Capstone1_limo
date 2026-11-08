#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <cmath>
#include <chrono>
// Prescribed kinematic motion for sensor-response tests, not a vehicle dynamics model.
class SmoothMotion : public ignition::gazebo::System,
 public ignition::gazebo::ISystemConfigure,
 public ignition::gazebo::ISystemPreUpdate {
 ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
 double cx=0,cy=0,rx=1,ry=1,z=.35,period=20,phase=0;
 public:
 void Configure(const ignition::gazebo::Entity &e,
 const std::shared_ptr<const sdf::Element> &s,
 ignition::gazebo::EntityComponentManager &,ignition::gazebo::EventManager &) override {
 model=ignition::gazebo::Model(e);
 cx=s->Get<double>("cx");cy=s->Get<double>("cy");
 rx=s->Get<double>("rx");ry=s->Get<double>("ry");z=s->Get<double>("z");
 period=s->Get<double>("period");phase=s->Get<double>("phase");
 }
 void PreUpdate(const ignition::gazebo::UpdateInfo &info,
 ignition::gazebo::EntityComponentManager &ecm) override {
 if(info.paused)return;
 double t=std::chrono::duration<double>(info.simTime).count();
 double a=phase+2*M_PI*t/period;
 double yaw=std::atan2(ry*std::cos(a),-rx*std::sin(a));
 model.SetWorldPoseCmd(ecm,{cx+rx*std::cos(a),cy+ry*std::sin(a),z,0,0,0});
 }
};
IGNITION_ADD_PLUGIN(SmoothMotion,ignition::gazebo::System,
 SmoothMotion::ISystemConfigure,SmoothMotion::ISystemPreUpdate)
