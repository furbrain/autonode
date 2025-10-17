import rclpy
from autonode import Node, DynamicParameter, Parameter, subscription, service
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool

class Test_Node(Node):
    name: str = "fred"
    age: int = 42
    alpha: int = Parameter(default=123, description="A described parameter")
    beta: str = DynamicParameter(default="hi", description="a dynamic parameter")

    @subscription(ColorRGBA,"color",10)
    def color(self, msg: ColorRGBA):
        print(f"Color: {msg}, {self.beta}")

    @service(SetBool, "booly")
    def boolo(self, req: SetBool.Request, rsp: SetBool.Response):
        print(f"boolo: {req}")
        rsp.success = True
        rsp.message = "sorted"
        return rsp

def main(args=None):
    rclpy.init(args=args)
    node = Test_Node()
    node.run()

if __name__=="__main__":
    main()