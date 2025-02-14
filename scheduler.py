import rospy
from std_msgs.msg import String
import subprocess

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

RRT = "rrt.py"
PURE_PURSUIT = "pure_pursuit.py"
TIME_LAP = "time_lap.py"
WALL_FOLLOW = "wall_follow.py"
WAYPOINT_LOGGER = "waypoint_logger.py"
RRT_REAL_TIME = "rrt-real-time.py"
GMAPPING = "gmapping"
PURE_PURSUIT_RRT_FRIENDLY = "pure_pursuit-rrt-friendly.py"

realCar = False

if(realCar):
    lidarscan_topic = '/scan'
    drive_topic = "vesc/ackermann_cmd_mux/input/navigation"
else:
    lidarscan_topic = '/scan'
    drive_topic = '/nav'



class Scheduler:
    def __init__(self):
        rospy.init_node('scheduler', anonymous=True)
        self.task_queue = [(1, WALL_FOLLOW, TIME_LAP), (1, WALL_FOLLOW, TIME_LAP, WAYPOINT_LOGGER), (2, RRT_REAL_TIME, TIME_LAP, PURE_PURSUIT_RRT_FRIENDLY)]  # Liste des nœuds à lancer, le premier élément représente le nombre de tour pendant lesquelles ces tâches seront actives
        self.current_tasks = None

        # Publisher et Subscriber
        self.publisher = rospy.Publisher('scheduler', String, queue_size=10)
        self.subscriber = rospy.Subscriber('scheduler', String, self.callback)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped)

        rospy.loginfo("Scheduler prêt !")
        self.task_in_progress= {}
        self.run_next_task()

        


    def callback(self, msg):
        data = msg.data
        if self.current_tasks is None:
            #rospy.logwarn("Aucune tâche en cours.")
            return
        if data == "TIME_LAP" + str(self.current_tasks[0]):
            """Callback pour recevoir des messages sur le topic scheduler."""
            rospy.loginfo(f"Tâche terminée : {msg.data}")

            '''drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            #print("consigne angle : ", angle, "consigne vitesse : ", velocity)
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0
            self.drive_pub.publish(drive_msg)'''



            self.stop_car()
            for task in self.task_in_progress:                  #On kill toutes les taches en cours avant de passer aux suivantes
                process = self.task_in_progress[task]
                if task == GMAPPING:
                    subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "mymap"])  #si on avait entrepris de construire une map, on en profite pour la sauvegarder
                if process is not None:
                    process.terminate()
                    self.task_in_progress[task] = None
                    rospy.loginfo(f"Terminé avec : {task}")
            self.stop_car()
            
            self.run_next_task()

    def run_next_task(self):
        """Lance la tâche suivante dans la queue."""
        if self.task_queue:
            print("COUCOUUUUUUUUUUUUUUU")
            self.current_tasks = self.task_queue.pop(0)
            
            for task in self.current_tasks[1:]:
                #rospy.loginfo(f"Lancement de la tâche : {self.current_task}")
                # Lancer le nœud avec `rosrun` ou une autre commande ROS
                if task == RRT:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["RRT"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == PURE_PURSUIT:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["PURE_PURSUIT"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == TIME_LAP:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["TIME_LAP"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == WALL_FOLLOW:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["WALL_FOLLOW"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == WAYPOINT_LOGGER:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["WAYPOINT_LOGGER"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == GMAPPING:
                    rospy.loginfo("Tâche lancée : gmapping")
                    self.task_in_progress["GMAPPING"] = subprocess.Popen(["roslaunch", "gmapping", "slam_gmapping_pr2.launch"])
                elif task == RRT_REAL_TIME:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["RRT_REAL_TIME"] = subprocess.Popen(["rosrun", "autonav", task])
                elif task == PURE_PURSUIT_RRT_FRIENDLY:
                    rospy.loginfo(f"Tâche lancée : {task}")
                    self.task_in_progress["PURE_PURSUIT_RRT_FRIENDLY"] = subprocess.Popen(["rosrun", "autonav", task])
                else:
                    rospy.logwarn(f"La tâche {task} n'est pas reconnue.")
        else:
            rospy.loginfo("Toutes les tâches ont été exécutées.")
            self.current_tasks = None
            self.stop_car()

    def stop_car(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        #print("consigne angle : ", angle, "consigne vitesse : ", velocity)
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)


if __name__ == '__main__':
    try:
        scheduler = Scheduler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scheduler arrêté.")