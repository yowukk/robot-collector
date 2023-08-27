from math import sin, cos, pi, acos
from copy import deepcopy
import rospy
from detection_msgs.msg import AnnotationWithPose2D
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid

# Объявление констант для последующей работы
dist = 1.5
literal_side = dist / cos(pi / 8)

# Объявление переменных для последующей работы
x_previous = 0
y_previous = 0
theta_previous = 0
i = 0
square_size = 0
x_target = None
y_target = None


# Функция, публикующая просмотренные треугольники на карту (также вызывает функцию, которая отмечает вершины просмотренными)
def callback(msg):
    # Считывание данных из сообщений
    theta = msg.pose.theta
    x = msg.pose.x
    y = msg.pose.y

    # Подсчёт координат камеры
    x_cam = x + 0.06 * cos(theta)
    y_cam = y + 0.06 * sin(theta)
    # Подсчет координат точек треугольника
    x2 = x_cam + cos(theta - pi / 8) * literal_side
    y2 = y_cam + sin(theta - pi / 8) * literal_side
    x3 = x_cam + cos(theta + pi / 8) * literal_side
    y3 = y_cam + sin(theta + pi / 8) * literal_side
    
    # "Закрашивание" вершин, которые видны из данной точки
    coloring(x_cam, y_cam, x2, y2, x3, y3)

    # Обращение к глобальным переменным для дальнейшего использования
    global x_previous, y_previous, theta_previous
    
    # Проверка на движение робота и на то достиг ли он цели (точки, в которую его направили)
    if (x_target == None and y_target == None) or (0.1 > abs(x - x_target) and 0.1 > abs(y - y_target)) or (abs(x_previous - x) <= 0.01 and abs(y_previous - y) <= 0.01 and abs(theta - theta_previous) <= pi / 36):
      # Выбор новой цели для робота
      main_func(x, y)

    # Запоминание значений x и y
    x_previous, y_previous, theta_previous = x, y, theta

    
    # Публикация измененной карты
    coloring_pub.publish(map)

    # # Вывод сообщения в терминал
    # rospy.loginfo("map published")

# Функция, которая возвращает кватернионы для данного угла
def get_quaternion_from_eule(yaw):
    qz = sin(yaw / 2)
    qw = cos(yaw / 2)
    return  qz, qw

# Функция, которая возвращает угол, который должен быть присвоен роботу при данных координатах и цели 
def count_the_angle(x, y, x0, y0):
    len_vect = ((x0 - x) ** 2 + (y0 - y) ** 2) ** 0.5
    alpha = acos((x0 - x) / len_vect)
    if y0 >= y:
      return alpha
    return -alpha


# Функция, вычисляющая псевдовекторное произведение
def vect(x1, y1, x2, y2):
    return x1 * y2 - y1 * x2

# Функция, проверяющая лежит ли точка в треугольнике
def is_in_triangle(x0, y0, xa, ya, xb, yb, xc, yc):
    p = vect(xa - x0, ya - y0, xb - xa, yb - ya)
    q = vect(xb - x0, yb - y0, xc - xb, yc - yb)
    r = vect(xc - x0, yc - y0, xa - xc, ya - yc)
    if (p >= 0 and q >= 0 and r >= 0) or (p <= 0 and q <= 0 and r <= 0):
        return True
    return False

# Функция, преводящая координаты в пиксели
def from_coordinates_to_pixels(x, y):
    x = (x - x_start) // square_size
    y = (y - y_start) // square_size
    return int(x), int(y)

# Функция, преводящая пиксели в координаты
def from_pixels_to_coordinates(x, y):
    x = x * square_size + x_start
    y = y * square_size + y_start
    return x, y

# Функция, которая визуализирует точку, в которую отправился робот
def points_drawer(x_target, y_target):
    # Обращение к глобальной переменной для её изменения для присвоения уникальных id
    global i

    # Создание точки
    m = Marker()
    m.type = 8
    m.id = i; i += 1
    ## Точке задаются размеры
    m.scale.x = square_size 
    m.scale.y = square_size 
    m.scale.z = square_size
    ## Точке задается цвет
    m.color.r = 0
    m.color.g = 1
    m.color.b = 0
    m.color.a = 1

    ##  Точке задаются координаты
    m.points = (Point(),)
    m.pose.orientation.w = 1
    m.pose.position.x = x_target + square_size / 2
    m.pose.position.y = y_target + square_size / 2
    m.pose.position.z = 0
    m.header.frame_id = "map"

    # Создание объекта для публиции 
    annot_pose = MarkerArray(markers = [m]) 

    # Публикация треугольника на карте
    annot_pub.publish(annot_pose)

# Функция выбора новой цели для робота и для начала объезда занаво
def main_func(x, y):
    # Обращение к глобальным переменным для возможности изменить её
    global not_used, map
    
    # Проверка на то есть ли непройденные точки
    if len(not_used) == 0:
      # Вывод в терминал предепреждения о том, что вся карта просмотрена
      rospy.logwarn("0 points left")
      # Присвоение временному списку непройденных вершин и карте стартовое значение 
      not_used = deepcopy(start_not_used)
      map.data = start_map.copy()
      # Выбираем новую цель для робота и удаляем её
      target = not_used.pop()
      # Удаляем все точки с карты, тк начинаем новый объезд
      m = Marker()
      m.action = 3
      annot_pose = MarkerArray(markers = [m]) 
      annot_pub.publish(annot_pose)
    else:
      # Вывод в терминал сообщения о том сколько непросмотренных вершин осталось
      rospy.loginfo(f"{len(not_used)} points left")
      # Выбираем новую цель для робота и удаляем её
      target = not_used.pop()
    
    # Обращение к глобальным переменным для возможности изменить их
    global x_target, y_target
    # Перевод значений цели из пикселей в координаты
    x_target, y_target = from_pixels_to_coordinates(target[0], target[1])

    # Визуализация точки, которую попытается увидеть робот
    points_drawer(x_target, y_target)

    # Подсчёт угла вектора
    alpha = count_the_angle(x, y, x_target, y_target)
    # Подсчет кватернионов, чтобы точнее задать роботу место куда он должен приехать (с углом)
    qz, qw = get_quaternion_from_eule(alpha)

    x_target = x_target - dist / 2 * cos(alpha)
    y_target = y_target - dist / 2 * sin(alpha)

    # Подготовка точки к публикации
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    # Передача других координат, чтобы выбранная точка лежала внутри треугольника
    goal.pose.position.x = x_target
    goal.pose.position.y = y_target
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw
    
    # Публикация точки
    goal_pub.publish(goal)

# Функция, которая помечает просмотренными вершины, которые видны из данной точки
def coloring(x1, y1, x2, y2, x3, y3):
    # Обращение к глобальным переменным для возможности изменить их
    global map, not_used

    # Перевод координат точек треугольника в пиксели
    x1, y1 = from_coordinates_to_pixels(x1, y1)
    x2, y2 = from_coordinates_to_pixels(x2, y2)
    x3, y3 = from_coordinates_to_pixels(x3, y3)

    # Нахождение минимальных и максимальных x и y, для того чтобы перебрать значения в диапазоне от наименьшего из возможных значений до наибольшего
    min_x = min(x1, x2, x3)
    min_y = min(y1, y2, y3)
    max_x = max(x1, x2, x3)
    max_y = max(y1, y2, y3)

    # Перебор x и y
    for i in range(min_x, max_x + 1):
      for j in range(min_y, max_y + 1):
        # Если точка принадлежит треугольнику, то она удаляется из списка непросмотренных вершин, и точка на карте перекрашивается
        if is_in_triangle(j, i, y1, x1, y2, x2, y3, x3):
          try:
            not_used.remove((i, j))
            map.data[j * width + i] = 50
          except KeyError:
            pass
    # # В терминале публикуется сообщение
    # rospy.loginfo("coloring done")


rospy.init_node('my_node')

# Создание издатель для взаимодействия с картой
annot_pub = rospy.Publisher('/debug', MarkerArray, queue_size = 2)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
coloring_pub = rospy.Publisher('/coloring/map', OccupancyGrid, queue_size = 2)

# Получение карты
map = rospy.wait_for_message('/map', OccupancyGrid)

# Вывод сообщения в терминал
rospy.loginfo("map recieved")

# Инициализация глобальных переменных
map.data = list(map.data)
start_map = map.data.copy()
width = map.info.width
square_size = map.info.resolution
x_start = map.info.origin.position.x
y_start = map.info.origin.position.y

# Преобразование карты из списка в множество непройденных вершин
cnt = 0
row = 0
not_used = set()
start_not_used = set()
for i in map.data:
    if i == 0:
      start_not_used.add((cnt, row))
    cnt += 1
    if cnt == width:
        row += 1
        cnt = 0
# Присваиваем временному множеству непройденных вершин стартовое значение 
not_used = deepcopy(start_not_used)

# Выводим кол-во непройденных вершин
rospy.logwarn(len(not_used))

# Подписка на сообщения от робота
rospy.Subscriber('/detections', AnnotationWithPose2D, callback)
# Бесконечный цикл, в котором происходит ожидание сообщений от робота
rospy.spin()
