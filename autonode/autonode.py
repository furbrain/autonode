import asyncio
import signal
from dataclasses import dataclass
from typing import Any, Optional
from inspect import getmembers, ismethod

import rclpy.node
from rclpy.callback_groups import CallbackGroup
from rclpy.node import SetParametersResult, ParameterDescriptor
from rclpy.qos import QoSProfile, qos_profile_services_default
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_overriding_options import QoSOverridingOptions



@dataclass
class Parameter:
    default: Any = None
    description: str = ""

class DynamicParameter(Parameter):
    ...

def service(srv_type,
            srv_name: str,
            *,
            qos_profile: QoSProfile = qos_profile_services_default,
            callback_group: Optional[CallbackGroup] = None):
    def wrapper(func):
        func.service_args = {"srv_type": srv_type,
                             "srv_name": srv_name,
                             "qos_profile": qos_profile,
                             "callback_group": callback_group}
        return func
    return wrapper

def subscription(msg_type,
                 topic: str,
                 qos_profile: QoSProfile,
                 *,
                 callback_group: Optional[CallbackGroup] = None,
                 event_callbacks: Optional[SubscriptionEventCallbacks] = None,
                 qos_overriding_options: Optional[QoSOverridingOptions] = None,
                 raw=False):
    def wrapper(func):
        func.subscription_args = {"msg_type": msg_type,
                             "topic": topic,
                             "qos_profile": qos_profile,
                             "callback_group": callback_group,
                             "event_callbacks": event_callbacks,
                             "qos_overriding_options": qos_overriding_options,
                             "raw": raw}
        return func
    return wrapper

class Node(rclpy.node.Node):
    def __init__(self, name=""):
        # set name to specified or classname (without "node" at the end)
        if not name:
            name = self.__class__.__name__
            if name.upper().endswith("NODE"):
                name = name[:-4]
                name = name.rstrip("_")
        super().__init__(name)

        #set up class vars as params
        self._dynamic_params = []
        for var, tp in self.__class__.__annotations__.items():
            if var.startswith("__"): # skip dunder items
                continue
            val = getattr(self, var, None)
            desc = ParameterDescriptor(read_only=True)
            if isinstance(val, Parameter):
                default = val.default
                if val.description:
                    desc.description = val.description
            else:
                default = val
            if isinstance(val, DynamicParameter):
                self._dynamic_params.append(var)
                desc.read_only = False
            self.declare_parameter(var, default, descriptor=desc)
            setattr(self, var, self.get_parameter(var).value)
        self.add_on_set_parameters_callback(self._on_param_change)

        # create service and subscriptions
        for var, val in getmembers(self, predicate=ismethod):
            if var.startswith("__"): # skip dunder methods
                continue
            if hasattr(val,"service_args"):
                setattr(self, var+"_service", self.create_service(callback=getattr(self, var),
                                                                  **val.service_args))
            if hasattr(val, "subscription_args"):
                setattr(self, var+"_sub", self.create_subscription(callback=getattr(self, var),
                                                                   **val.subscription_args))
        self._running = True
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    def _on_param_change(self, params):
        for param in params:
            if param.name in self._dynamic_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Parameter '{param.name}' updated → {param.value}")
        return SetParametersResult(successful=True)

    def _sig_handler(self, sig, frame):
        self.get_logger().info(f"Received signal {sig}, shutting down gracefully...")
        self._running = False

    def on_loop(self):
        """
        This function will be run once every time through the loop while node is running
        :return:
        """
        pass

    async def on_loop_async(self):
        """
        This function will be run once every time through the loop while node is running
        async
        :return:
        """
        pass

    def run(self):
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                self.on_loop()
        except Exception as e:
            self.get_logger().error(f"Unhandled exception: {e}")
        finally:
            self.cleanup()
            self.destroy_node()
            rclpy.shutdown()

    async def run_async(self):
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                await asyncio.sleep(0.01)
                await self.on_loop_async()
        except Exception as e:
            self.get_logger().error(f"Unhandled exception: {e}")
        finally:
            self.cleanup()
            self.destroy_node()
            rclpy.shutdown()

    def cleanup(self):
        """
        Override this function if there are any resources which need releasing before shutting down
        :return:
        """
