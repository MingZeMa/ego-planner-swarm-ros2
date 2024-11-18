#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "pose_tool.h"

namespace rviz
{

Pose3DTool::Pose3DTool()
  : Tool()
  , arrow_(nullptr)
{
}

Pose3DTool::~Pose3DTool()
{
  arrow_array_.clear();
}

void Pose3DTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = State::Position;
}

void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  int flags = 0;
  static Ogre::Vector3 ang_pos;
  static double initz;
  static double prevz;
  static double prevangle;
  const double z_scale = 50;
  const double z_interval = 0.5;
  Ogre::Quaternion orient_x =
    Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

  if (event.leftDown())
  {
    assert(state_ == State::Position);
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (rviz_rendering::getPointOnPlaneFromWindowXY(
          event.viewport, ground_plane, event.x, event.y, intersection))
    if (calculate3DPositionFromWindowXY(event.x, event.y, cur_pos))      
    {
      pos_ = intersection;
      arrow_->setPosition(pos_);
      state_ = State::Orientation;
      flags |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == State::Orientation)
    {
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (rviz_rendering::getPointOnPlaneFromWindowXY(
            event.viewport, ground_plane, event.x, event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setOrientation(Ogre::Quaternion(orient_x));
        if (event.right())
          state_ = State::Height;
        initz = pos_.z;
        prevz = event.y;
        prevangle = angle;
        flags |= Render;
      }
    }
    if (state_ == State::Height)
    {
      double z = event.y;
      double dz = z - prevz;
      prevz = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition(pos_);

      arrow_array_.clear();
      int cnt = ceil(fabs(initz - pos_.z) / z_interval);
      for (int k = 0; k < cnt; k++)
      {
        auto arrow__ = std::make_shared<rviz_rendering::Arrow>(
          scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        arrow__->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        arrow__->getSceneNode()->setVisible(true);
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = initz - ((initz - pos_.z > 0) ? 1 : -1) * k * z_interval;
        arrow__->setPosition(arr_pos);
        arrow__->setOrientation(
          Ogre::Quaternion(Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z) *
          orient_x);
        arrow_array_.push_back(arrow__);
      }
      flags |= Render;
    }
  }
  else if (event.leftUp())
  {
    if (state_ == State::Orientation || state_ == State::Height)
    {
      arrow_array_.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished | Render);
    }
  }

  return flags;
}

}  // namespace rviz_plugins
