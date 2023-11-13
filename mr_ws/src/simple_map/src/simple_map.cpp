#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

//глобальная переменная - публикатор сообщения карты
ros::Publisher mapPub;

//глоабльный указатель на tfListener, который будет проинициализирован в main
tf::TransformListener *tfListener;

//имя для СК карты
std::string map_frame;

//разрешение карты
double map_resolution = 0.1;
//размер карты в клетках
int map_width = 2000;
int map_height = 2000;


//создаем сообщение карты
nav_msgs::OccupancyGrid map_msg;

void prepareMapMessage(nav_msgs::OccupancyGrid& map_msg)
{
    map_msg.header.frame_id = map_frame;
    
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.info.resolution = map_resolution;
    // фиксированное положение начала карты (левый нижний угол)
    map_msg.info.origin.position.x = - map_width * map_resolution /2.0;
    map_msg.info.origin.position.y = - 10.0;

    // изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
    map_msg.data.resize(map_height*map_width, -1);
}

bool determineScanTransform(tf::StampedTransform& scanTransform,
                            const ros::Time& stamp,
                            const std::string& laser_frame)
{
    try
    {
        if ( ! tfListener->waitForTransform(map_frame,
                                            laser_frame,
                                            stamp,
                                            ros::Duration(0.1)) )
        {
          ROS_WARN_STREAM("no transform to scan "<<laser_frame);
          return false;
        }
        tfListener->lookupTransform(map_frame,
                                    laser_frame,
                                    stamp,
                                    scanTransform);

    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR_STREAM("got tf exception "<<e.what());
        return false;
    }
    return true;
}

float log2p(float p)
{
    return log(p/(1-p));
}

float calc_p(float l)
{
    return 1 - (1/(1+exp(l)));
}

int get_map(float l)
{
    float p = calc_p(l);
if (p < 0.4) { 
        return 0;
    }
else if (p > 0.6) { return 100; }
else return 50;
}

// Построение карты только по сырым данным с лидара: точки могут принимать значения 0 (пустая) или 100 (препятствие)
void laserCallback(const sensor_msgs::LaserScan& scan)
{
    tf::StampedTransform scanTransform;
    const std::string& laser_frame = scan.header.frame_id;
    const ros::Time& laser_stamp = scan.header.stamp;
    if (!determineScanTransform(scanTransform, laser_stamp, laser_frame)) {
        return;
    }

    map_msg.header.stamp = laser_stamp;

    //положение центра дальномера в СК дальномера
    tf::Vector3 zero_pose(0, 0, 0);
    //положение дальномера в СК карты
    tf::Vector3 scan_pose = scanTransform(zero_pose);
    ROS_INFO_STREAM("scan pose "<<scan_pose.x()<<" "<<scan_pose.y());

    //индексы карты, соответствующие положению центра лазера
    int y = (scan_pose.y() - map_msg.info.origin.position.y ) / map_resolution;
    int x = (scan_pose.x() - map_msg.info.origin.position.x ) / map_resolution;
    ROS_INFO_STREAM("publish map "<<x<<" "<<y);
    // в клетку карты записываем значение 100
    map_msg.data[ y* map_width + x] = 0;

    int size_ranges = scan.ranges.size();
    ROS_INFO_STREAM("Publish map"<<size_ranges);
    float curr_angle = scan.angle_min;
    float delta_angle = scan.angle_increment;
    int i=0;
    while(curr_angle < scan.angle_max)
    {
        float range = scan.ranges[i];
        if(range>scan.range_min && range < scan.range_max )
        {
            float h = scan.range_min;
            while(h<=range)
            {
                tf::Vector3 h_pose(h*cos(curr_angle), h*sin(curr_angle),0);
                tf::Vector3 h_map = scanTransform(h_pose);
                int h_y = (h_map.y() - map_msg.info.origin.position.y )/map_resolution;
                int h_x = (h_map.x() - map_msg.info.origin.position.x )/map_resolution;
                float p = 0.5;
                if (abs(range - h) < 0.1) {
                    p = 1.0;
                    }
                    else if (range - h > 0.1) {
                    p = 0.0;
                    }

                    map_msg.data[ h_y* map_width + h_x] = 0;
                    h += 0.01;
                }
            
            tf::Vector3 h_pose(h*cos(curr_angle), h*sin(curr_angle),0);
            tf::Vector3 h_map = scanTransform(h_pose);
            int h_y = (h_map.y() - map_msg.info.origin.position.y )/map_resolution;
            int h_x = (h_map.x() - map_msg.info.origin.position.x )/map_resolution;
            map_msg.data[ h_y* map_width + h_x] = 100; 
            }
            
        i++;
        curr_angle += delta_angle;
    }
    // публикуем сообщение с построенной картой
    mapPub.publish(map_msg);
}

// Построение вероятностной карты с помощью рекурсивного алгоритма Байеса
// void laserCallback(const sensor_msgs::LaserScan& scan)
// {
//     tf::StampedTransform scanTransform;
//     const std::string& laser_frame = scan.header.frame_id;
//     const ros::Time& laser_stamp = scan.header.stamp;
//     if (!determineScanTransform(scanTransform, laser_stamp, laser_frame)) {
//         return;
//     }

//     map_msg.header.stamp = laser_stamp;

//     //положение центра дальномера в СК дальномера
//     tf::Vector3 zero_pose(0, 0, 0);
//     //положение дальномера в СК карты
//     tf::Vector3 scan_pose = scanTransform(zero_pose);
//     ROS_INFO_STREAM("scan pose "<<scan_pose.x()<<" "<<scan_pose.y());

//     //индексы карты, соответствующие положению центра лазера
//     int y = (scan_pose.y() - map_msg.info.origin.position.y ) / map_resolution;
//     int x = (scan_pose.x() - map_msg.info.origin.position.x ) / map_resolution;
//     ROS_INFO_STREAM("publish map "<<x<<" "<<y);
//     // в клетку карты записываем значение 100
//     map_msg.data[ y* map_width + x] = 0;

//     int size_ranges = scan.ranges.size();
//     ROS_INFO_STREAM("Publish map"<<size_ranges);
//     float curr_angle = scan.angle_min;
//     float delta_angle = scan.angle_increment;
//     int i=0;
//     while(curr_angle < scan.angle_max)
//     {
//         float range = scan.ranges[i];
//         if(range>scan.range_min && range < scan.range_max )
//         {
//             float h = scan.range_min;
//             while(h<=range)
//             {
//                 tf::Vector3 h_pose(h*cos(curr_angle), h*sin(curr_angle),0);
//                 tf::Vector3 h_map = scanTransform(h_pose);
//                 int h_y = (h_map.y() - map_msg.info.origin.position.y )/map_resolution;
//                 int h_x = (h_map.x() - map_msg.info.origin.position.x )/map_resolution;
//                 float p = 0.5;
//                 if (abs(range - h) < 0.1) {
//                     p = 1.0;
//                     }
//                     else if (range - h > 0.1) {
//                     p = 0.0;
//                     }

//                     float log_prev = log2p(float(map_msg.data[ h_y* map_width +h_x])/100);
//                     float log_free = log2p(0.5);
//                     float log_inv = log2p(p);
//                     float log_ti = log_inv + log_prev - log_free;
//                     map_msg.data[ h_y* map_width + h_x] = get_map(log_ti);
//                     h += 0.01;
//                 }
            
//             }
            
//         i++;
//         curr_angle += delta_angle;
//     }
//     // публикуем сообщение с построенной картой
//     mapPub.publish(map_msg);
// }


int main(int argc, char **argv)
{
  /**
   * Инициализация системы сообщений ros
   * Регистрация node с определенным именем (третий аргумент функции)
   * Эта функция должна быть вызвана в первую очередь
   */
  ros::init(argc, argv, "control_node");

  /**
   * NodeHandle  - объект через который осуществляется взаимодействие с ROS:
   * передача сообщений
   * регистрация коллбаков (функций обработки сообщений)
   */
  ros::NodeHandle node("~");

  //читаем параметры
  map_frame = node.param<std::string>("map_frame", "odom");
  map_resolution = node.param("map_resolution", map_resolution);
  map_width = node.param("map_width", map_width);
  map_height = node.param("map_height", map_height);

  //создание объекта tf Listener
  tfListener = new tf::TransformListener;

  // Подписываемся на данные дальномера
  ros::Subscriber laser_sub = node.subscribe("/scan", 100, laserCallback);

  //объявляем публикацию сообщений карты
  //Используем глобальную переменную, так как она понядобится нам внутр функции - обработчика данных лазера

  mapPub = node.advertise<nav_msgs::OccupancyGrid>("/simple_map", 10);


  //заполняем информацию о карте - готовим сообщение
  prepareMapMessage(map_msg);
   /**
   * ros::spin() функция внутри которой происходит вся работа по приему сообщений
   * и вызову соответствующих обработчиков . Вся обработка происходит из основного потока
   * (того, который вызвал ros::spin(), то есть основного в данном случае)
   * Функция будет завершена, когда подьзователь прервет выполнение процесса с Ctrl-C
   *
   */
  ros::spin();

  return 0;
}
