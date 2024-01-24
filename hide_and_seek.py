import random
import rospy
import tf
import numpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Variabila globala pentru a stoca locurile de ascuns
hiding_spots = []

# Functie callback pentru datele scanarii laser
def laser_callback(data):
    global hiding_spots
    hiding_spots = []  # Resetam lista de ascunzatori
    ranges = data.ranges  # Luam datele de distanta de la scanarea laser
    # Codul pentru datele de distanta si pentru identificarea potentialelor ascunzatori
    for i in range(len(ranges)):
        # Verificam daca distanta nu depaseste o anumita limita (0.5m)
        if ranges[i] > 0.5:
            # Convertim datele de distanta si cele unghiulare in coordonate carteziene
            x = ranges[i] * numpy.cos(data.angle_min + i * data.angle_increment)
            y = ranges[i] * numpy.sin(data.angle_min + i * data.angle_increment)
            # Verificam daca punctul se afla la o anumita distanta de robot (1m)
            if numpy.sqrt(x**2 + y**2) < 1:
                # Adaugam punctul la lista de ascunzatori
                hiding_spots.append((x, y))

# Functie callback pentru datele odometrice
def odom_callback(data):
    global hiding_spots
    if hiding_spots:
        # Se alege o ascunzatoare aleatoare
        spot = random.choice(hiding_spots)
        # Codul pentru a naviga la punctul ales
        move_to_spot(spot)
    else:
        print("No suitable hiding spots found.")

# Functia pentru deplasarea robotului la punctul ales
def move_to_spot(spot):
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Se ia pozitia curenta a robotului
        current_pos = get_current_position()
        # Calculam distanta si unghiul robotului fata de punct
        distance = numpy.sqrt((spot[0] - current_pos[0])**2 + (spot[1] - current_pos[1])**2)
        angle = numpy.arctan2(spot[1] - current_pos[1], spot[0] - current_pos[0])
        # Creeam un mesaj Twist pentru a misca robotul
        twist = Twist()
        # Setam velocitatea lineara si unghiulara bazata pe distanta si unghiul robotului
        # fata de ascunzatoare
        twist.linear.x = min(distance, 0.1)  # Limitam viteza lineara la maximul de 0.1 m/s
        twist.angular.z = angle
        pub.publish(twist)
        rate.sleep()
        if distance < 0.1:  # Daca robotul ajunge la ascunzatoare
            break

# Functia pentru a obține poziția curentă
def get_current_position():
    position = (0, 0)
    tf_listener = tf.TransformListener()  # Initializam un obiect de tip tf.TransformListener
    try:
        # Ne folosim de functia lookupTransform a obiectului pentru a transforma
        # datele din tipul map in tipul base_footprint.
        (trans, rot) = tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        # Aceasta returneaza doua liste. Prima lista reprezinta transformarea liniara din
        # primul tip in al doilea, iar cea de a doua reprezinta rotatia.
        position = (trans[0], trans[1])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    return position

# Functia pentru a indica ca ascunzatoarea a fost gasita
def indicate_found():
    # Codul pentru a se roti pentru a indica ca a gasit ascunzatoarea
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.angular.z = 1
        pub.publish(twist)
        print("Found!\n")
        rospy.sleep(0.1)

# Functia main pentru a se juca hide-and-seek
def play_hide_and_seek():
    # Initializam nodurile si subscriberil
    rospy.init_node("hide_and_seek")
    rospy.Subscriber("scan", LaserScan, laser_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    # Folosim spin pentru a mentine programul in functiune
    rospy.spin()

if __name__ == "__main__":
    play_hide_and_seek()
    try:
        pass
    except rospy.ROSInterruptException:
        pass
