#ifndef RVIZ_POSE_TOOL_H
#define RVIZ_POSE_TOOL_H

#include <memory>
#include <vector>

#include <OgreVector3.h>
#include <QCursor>

#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"

namespace rviz
{
class DisplayContext;


class Pose3DTool : public rviz_common::Tool
{
public:
  Pose3DTool();
  virtual ~Pose3DTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrow_array_;

  enum class State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;
};

} // namespace rviz_plugins

#endif // RVIZ_POSE_TOOL_H
