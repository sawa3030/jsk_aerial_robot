#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest


def make_box_inertia(mass, size_x, size_y, size_z):
    return {
        "mass": mass,
        "com": (0.0, 0.0, 0.0),
        "ixx": mass * (size_y ** 2 + size_z ** 2) / 12.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyy": mass * (size_x ** 2 + size_z ** 2) / 12.0,
        "iyz": 0.0,
        "izz": mass * (size_x ** 2 + size_y ** 2) / 12.0,
    }


DEFAULT_TRANSFORM = {
    "translation": (0.0, 0.35, 0.0),
    "rotation": (0.0, 0.0, 0.0, 1.0),
}

OBJECT_CONFIGS = (
    {
        "module_name": "object1",
        "parent_link_name": "soft_link13",
        "transform": DEFAULT_TRANSFORM,
        "inertia": make_box_inertia(0.280, 0.70, 0.23, 0.18),
    },
)


class AddThreeExtraModulesNode(object):
    def __init__(self):
        self.srv_name = rospy.get_param("~service_name", "add_extra_module")
        rospy.loginfo("Waiting for service: %s", self.srv_name)
        rospy.wait_for_service(self.srv_name)
        self.add_extra_module = rospy.ServiceProxy(self.srv_name, AddExtraModule)

    def _build_request(self, config):
        req = AddExtraModuleRequest()
        req.action = AddExtraModuleRequest.ADD
        req.module_name = config["module_name"]
        req.parent_link_name = config["parent_link_name"]

        translation = config["transform"]["translation"]
        rotation = config["transform"]["rotation"]
        req.transform.translation.x = translation[0]
        req.transform.translation.y = translation[1]
        req.transform.translation.z = translation[2]
        req.transform.rotation.x = rotation[0]
        req.transform.rotation.y = rotation[1]
        req.transform.rotation.z = rotation[2]
        req.transform.rotation.w = rotation[3]

        inertia = config["inertia"]
        req.inertia.m = inertia["mass"]
        req.inertia.com.x = inertia["com"][0]
        req.inertia.com.y = inertia["com"][1]
        req.inertia.com.z = inertia["com"][2]
        req.inertia.ixx = inertia["ixx"]
        req.inertia.ixy = inertia["ixy"]
        req.inertia.ixz = inertia["ixz"]
        req.inertia.iyy = inertia["iyy"]
        req.inertia.iyz = inertia["iyz"]
        req.inertia.izz = inertia["izz"]
        return req

    def add_module(self, config):
        req = self._build_request(config)
        try:
            resp = self.add_extra_module(req)
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed for %s: %s", req.module_name, exc)
            return False

        if resp.status:
            rospy.loginfo(
                "Added extra module: %s (parent: %s)",
                req.module_name,
                req.parent_link_name,
            )
        else:
            rospy.logwarn(
                "Failed to add extra module: %s (parent: %s)",
                req.module_name,
                req.parent_link_name,
            )
        return bool(resp.status)

    def run(self):
        rospy.sleep(0.5)
        results = []
        for config in OBJECT_CONFIGS:
            results.append((config["module_name"], self.add_module(config)))
            rospy.sleep(0.2)

        success_count = sum(1 for _, status in results if status)
        rospy.loginfo("Finished adding modules: %d/%d succeeded", success_count, len(results))


def main():
    rospy.init_node("add_three_extra_modules")
    node = AddThreeExtraModulesNode()
    node.run()


if __name__ == "__main__":
    main()
