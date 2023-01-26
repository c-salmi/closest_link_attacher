# Closest Link Attacher (a gazebo plugin)

This plugin is provides a rosservice that can attach the closest link in the world to a given other link (if that link is within the minimum distance threshold).

A possible usecase for this plugin is to fake a vacuum gripper in gazebo, without having to explicitly know the name of the link you want to attach to.

## Limitations:
- Supports only single layer of nested models, if the link is part of a model that is nested more than that it will not be found.
- Cannot use two attachments at the same time, because we reuse the single reference to physics::JointPtr
