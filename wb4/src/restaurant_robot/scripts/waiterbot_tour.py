#!/usr/bin/env python3
"""
WaiterBot Tour — ROS 2 Humble — Frankfurt UAS
Robot spawns at (0, 3.5) facing west.
Front tables y=1.0 → approach y=2.5
Back  tables y=-3.0 → approach y=-1.2
"""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatus

HOME   = (0.0,  3.5,  0.0, "Service Station")
FRONT_Y =  2.5
BACK_Y  = -1.2

TABLES = {
    1: (-5.0, FRONT_Y, -1.5707, "Table 1  (front left)"),
    2: ( 0.0, FRONT_Y, -1.5707, "Table 2  (front centre)"),
    3: ( 5.0, FRONT_Y, -1.5707, "Table 3  (front right)"),
    4: (-5.0, BACK_Y,  -1.5707, "Table 4  (back left)"),
    5: ( 0.0, BACK_Y,  -1.5707, "Table 5  (back centre)"),
    6: ( 5.0, BACK_Y,  -1.5707, "Table 6  (back right)"),
}
TOUR_ORDER = [1, 2, 3, 6, 5, 4]

def make_pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp    = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p

def bar(): print("  " + "─" * 48)

class WaiterBot(Node):
    def __init__(self):
        super().__init__('waiterbot_tour')
        self._nav     = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def stop(self):
        msg = Twist()
        for _ in range(20):
            self._cmd_vel.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        print("  Connecting to Nav2...")
        self._nav.wait_for_server(timeout_sec=60.0)
        print("  Nav2 found — waiting 15s for AMCL to converge...")
        time.sleep(15.0)
        # Kill any residual Gazebo drift
        print("  Stopping robot drift...")
        self.stop()
        time.sleep(2.0)
        self.stop()
        print("  ✅  Ready\n")

    def go_to(self, x, y, yaw, label, silent=False):
        if not silent:
            print(f"      → ({x:.1f}, {y:.1f})")
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(self, x, y, yaw)
        fut = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle or not handle.accepted:
            return False
        res_fut = handle.get_result_async()
        while not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        ok = res_fut.result().status == GoalStatus.STATUS_SUCCEEDED
        self.stop()
        return ok

    def visit_table(self, table_id, step, total):
        x, y, yaw, label = TABLES[table_id]
        print(f"\n  [{step}/{total}]  ➤  {label}")
        print(f"      Target: ({x:.1f}, {y:.1f})")
        print(f"      ⏳  Navigating...")
        # Go to aisle centre first, then slide to table
        aisle_y = FRONT_Y if y > 0 else BACK_Y
        self.go_to(0.0, aisle_y, 0.0, "aisle", silent=True)
        ok = self.go_to(x, y, yaw, label, silent=True)
        if ok:
            print(f"      ✅  Arrived at {label}")
        else:
            print(f"      ❌  Failed: {label}")
        return ok

    def full_tour(self, table_ids):
        total   = len(table_ids) + 1
        results = {}
        bar()
        print(f"  Tour: Home → {' → '.join(f'T{t}' for t in table_ids)} → Home")
        bar()
        step = 1
        for t in table_ids:
            ok = self.visit_table(t, step, total)
            results[t] = ok
            step += 1
            if ok: time.sleep(0.5)

        # Return home via front aisle
        print(f"\n  [{step}/{total}]  ➤  {HOME[3]}")
        print(f"      Target: ({HOME[0]:.1f}, {HOME[1]:.1f})")
        print(f"      ⏳  Navigating...")
        self.go_to(0.0, FRONT_Y, 0.0, "aisle", silent=True)
        ok_home = self.go_to(*HOME[:3], HOME[3], silent=True)
        print(f"      {'✅  Returned to '+HOME[3] if ok_home else '❌  Failed to return home'}")

        bar()
        print(f"\n  TOUR SUMMARY")
        bar()
        for t in table_ids:
            icon = "✅  VISITED" if results[t] else "❌  MISSED "
            print(f"    {icon}  {TABLES[t][3]}")
        print(f"    {'✅  RETURNED' if ok_home else '❌  MISSED  '}  {HOME[3]}")
        bar()
        arrived = sum(1 for v in results.values() if v)
        print(f"\n  Tables visited: {arrived}/{len(table_ids)}")
        if arrived == len(table_ids) and ok_home:
            print("  🎉  Full tour completed!")
        else:
            print(f"  ⚠️   {len(table_ids)-arrived} stop(s) missed.")
        bar()

def main():
    rclpy.init()
    bot = WaiterBot()
    bot.wait_ready()
    args  = sys.argv[1:]
    start = time.time()

    print("  ╔══════════════════════════════════════════╗")
    print("  ║   🤖  WAITERBOT — Frankfurt UAS          ║")
    print("  ║   Autonomous Restaurant Navigation       ║")
    print("  ╚══════════════════════════════════════════╝\n")

    if not args:
        bot.full_tour(TOUR_ORDER)
    elif len(args)==1 and args[0].isdigit() and 1<=int(args[0])<=6:
        t = int(args[0])
        bot.visit_table(t, 1, 2)
        time.sleep(0.5)
        bot.go_to(0.0, FRONT_Y, 0.0, "aisle", silent=True)
        bot.go_to(*HOME[:3], HOME[3])
    elif all(a.isdigit() and 1<=int(a)<=6 for a in args):
        bot.full_tour([int(a) for a in args])
    else:
        print("  Usage:")
        print("    python3 waiterbot_tour.py        # full tour")
        print("    python3 waiterbot_tour.py 3      # table 3 only")
        print("    python3 waiterbot_tour.py 1 2 3  # custom")

    elapsed = int(time.time()-start)
    print(f"\n  ⏱   Total: {elapsed//60}m {elapsed%60}s")
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
