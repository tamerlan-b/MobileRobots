## Структура репозитория
* Папка docker - папка со скриптами для запуска docker-контейнера
  * docker/build_docker - скрипт для построения докер-образа 
  * docker/start_docker - скрипт для запуска докер-образа
  * docker/attach_docker - скрипт для подключения к командной строке внутри докер-образа
  * docker/.bashrc - скрипт, который выполняется при подключении к командной строке (инициализация ros и инициализация проекта)
  * docker/dockerfile - папка с описанием докер образа (Dockerfile)
* Папка mr_ws - рабочая папка с проектами ros. В докер-образе эта папка монтируется как `/home/$USER/local/workspace/mr_ws`
Сборка проекта из под docker опциональна. Можно собирать в нативном окружении, обеспечив установку зависимостей проектов. В докер образе эти зависимости уже установлены, что делает жизнь проще. В докере установлена ОС Ubuntu и ROS melodic. 
## Использование docker 
0. Установливаем Docker по инструкции: https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#os-requirements. Настраиваем Docker для работы с ним без root доступа (без sudo): https://docs.docker.com/install/linux/linux-postinstall/
1. Клонируем репозиторий
```bash
$git clone https://github.com/AndreyMinin/MobileRobots.git <папка для размещения проекта>
```
Следующие команды выполняем из папки, куда клонирован репозиторий (все команды должны выполняться из-под обычного пользователя)

2. Собираем образ (эта команда выполняется один раз, а также при изменении файла образа Dockerfile) 
``` bash
$ ./docker/build_docker
```
Выполнение займет какое-то время, требуемое для скачивания базового образа и установки пакетов

3. Запускаем контейнер (выполняется каждый раз для запуска контейнера для работы с проектами) Также при выполнении этой команды в контейнере создается пользователь с именем как у текущего пользователя системы ($USER). Текущая директория (папка проекта) монтируется в докер по адресу `/home/<имя пользователя>/local/workspace`. Вся домашняя папка текущего пользователя в базовой системе монтируется в докер по адресу `/host`
``` bash	
$ ./docker/start_docker
```
4. Подсоединяем консоль к контейнеру. (можно выполнить несколько раз из разных терминалов - сколько нужно терминалов внутри докера)
``` bash
$ ./docker/attach_docker
```
В результате выполнения этой команды текущий терминал оказывается подключен к докеру - оказываемся в командной строке, выполняющейся внутри докера. Все команды, выполняемые из этой консоли выполняются внутри докера (изолировано от нашей системы). Изначально мы оказываемся в ROS воркспейсе(`/home/<имя пользователя>/local/workspace/mr_ws`) нашего проекта. В первый раз необходимо собрать проект с помощью команды `catkin_make` и после сборки проинициализировать workspace.При повторных запусках консоли - инициализация workspace осуществляется автоматически. 
``` bash
$ catkin_make
$ source devel/setup.bash
```

5. Запуск проектов возможен с помощью 'roslaunch' и 'rosrun'. Также можно запустить первый проект с помощью скрпита `start.sh`. В последнем случае должно появиться окно симулятора Gazebo и окно rqt - в котором с помощью плагина publish message можно задать скорость движения робота

6. Для возврата из докера используется команда (можно просто закрыть терминал)
``` bash
$ exit
``` 

После этого терминал окажется в основной системе.

7. Для просмотра текущих запущенных образов докера можно использовать команду в основной системе:
``` bash
$ docker ps
```

8. Для того, чтобы остановить докер нуно выполнить команду в основной системе:
``` bash
$ ./docker/stop_docker
```
Все изменения в докере (установка новых deb или python пактов, изменения файлов) кроме тех, что сделаны в папке проекта `/home/<имя пользователя>/local/workspace` после остановки образа будут утеряны. При выключении/перезагрузке компьютера все образы будут остановлены.


	
