
# Practica Gazebo Simulator 

Este segundo proyecto de la asignatura de Simuladores de Robots se divide en dos partes:
- 10% plugin de C++ para Gazebo en el que al robot se le da  un 
punto como destino y este va hacia ese punto. 
- Extras:
    + Implementación de ROS1 y Python en Gazebo
    + Utilización de un modelo diferente para el robot, en este caso TurtleBot3
    + Navegación con el robot utilizando el Navigation Stack
    + Implementación de un algoritmo de exploración de mapas desconocidos
    + Pruebas de exploración en cuatro mapas diferentes





## Authors
- [Jaime Mas Santillán](https://www.github.com/TheArmega)


## Index

    1. Movimiento básico mediante plugin
    2. Modelo TurtleBot3
    3. Algoritmo de exploración
    4. Resultados algoritmo de exploración
## Movimiento básico mediante plugin
En esta primera parte de la práctica se nos pedía que una plataforma robótica fuese de un punto A a un punto B del mapa sin que se encontrase en su camino con obstáculos. Para ellos he desarrollado un algoritmo que toma el punto actual y las coordenadas de un objetivo que se le pasan como parametro en el XML. Utiliza el algoritmo A* para calcular una ruta desde el nodo actual hasta el objetivo. Si no hay camino o el robot ya está en el objetivo, este se parará. El algoritmo se compone basicamente de:

- `Load`

```c++
void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MovePlugin plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MovePlugin plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";
                
            // Set the target position
            targetPosition = ignition::math::Vector3d(9.0, 16.0, 0); // Cambia la posición final según sea necesario
  
            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MovePlugin::OnUpdate, this));
        }

```

- `OnUpdate`
```c++
void OnUpdate()
        {
            // Get current pose
            ignition::math::Pose3d pose = model->WorldPose();
            
            // Calculate direction towards the target position
            ignition::math::Vector3d direction = targetPosition - pose.Pos();
            double distanceToTarget = direction.Length();
            direction.Normalize();

            // Calculate the desired orientation towards the target position
            double targetYaw = atan2(direction.Y(), direction.X());
            double currentYaw = pose.Rot().Yaw();
            double yawError = targetYaw - currentYaw;

            //Check if target has been reached
            if (distanceToTarget < 1.0)  {
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                printf("Objetivo alcanzado\n");
        
            // Check if within orientation tolerance
            } else if (std::abs(yawError) > 0.05) {
                // Apply proportional control to adjust the orientation
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                double maxAngularVelocity = 1.0; // Ajusta según sea necesario
                double angularVelocity = std::min(std::max(yawError, -maxAngularVelocity), maxAngularVelocity);
                printf("At: %f %f %f\n %f \n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), distanceToTarget);

                // Set the angular velocity
                model->SetAngularVel(ignition::math::Vector3d(0, 0, angularVelocity));
            } else {
                // Within orientation tolerance, stop angular velocity
                model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
                // Set the linear velocity
                leftJoint->SetVelocity(0, 2.0);
                rightJoint->SetVelocity(0, 2.0);
                printf("At: %f %f %f\n %f \n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), distanceToTarget);
            }
           
        }
```

- `XML`
```xml
<include>
      <uri>model://pioneer</uri>
      <pose>1.45 1.6 0 0 0 0</pose>
      <plugin name="move_plugin" filename="libmove_plugin.so">
        <goal_position>5.0 5.0 0.0</goal_position>
        <left_joint>left_wheel_hinge</left_joint>             
        <right_joint>right_wheel_hinge</right_joint> 
      </plugin> 
    </include> 
```

Con este script obtenemos el siguiente resultado:



## Modelo TurtleBot3
He probado a cambiar el modelo del robot al TurtleBot3, el resultado es el siguiente:


## Algoritmo de exploración
Para que el robot sea capaz de explorar un entorno desconocido he implementado un algoritmo en el que, mediante el uso del sensor LIDAR del TurtleBot3, este escanea el entorno y mediante el topic

`map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)`

en cada iteración recibe el nuevo mapa con los nuevos valores que ha obtenido el LIDAR, siendo -1 para zonas en las que no sabe que hay, 0 para zonas a las que puede ir y 100 para los obstáculos.

Para ello se recorre el mapa y se guardan en una lista los puntos a los que se puede navegar y alrededor de este punto hay puntos sin explorar:

`find_unknown_cells`
```python
def find_unknown_cells(map_data):
    global visitedNodes

    unknown_cells = [0, 0]
    if map_data is not None and map_data.info is not None:
        for y in range(map_data.info.height):
            for x in range(map_data.info.width):
                index = y * map_data.info.width + x
                if map_data.data[index] == 0 and (x, y) not in visitedNodes:
                    if map_data.data[(y - 2) * map_data.info.width + x] == -2 or map_data.data[(y + 2) * map_data.info.width + x] or map_data.data[y * map_data.info.width + x - 2] or map_data.data[y * map_data.info.width + x + 2]:
                        unknown_cells = [x, y]
                        visitedNodes.append((x, y))
                        if len(visitedNodes) > int(((map_data.info.width * map_data.info.height) * 0.05)/100):
                            return None
                        return unknown_cells
        return None

    return unknown_cells
```

Una vez que se han encontrado esos puntos que utilizamos como criterio nos ponemos a navegar hacia ellos:

`select_and_publish_goal`
```python
def select_and_publish_goal(map_data):
    global visitedNodes, aux_x, aux_y

    if map_data is not None:
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height

        unknown_cells = find_unknown_cells(map_data)
        
        if unknown_cells:
            random_x = unknown_cells[1]
            random_y = unknown_cells[0]

            print("Voy: ", random_x, random_y)
            print(visitedNodes)
            print(len(visitedNodes))

            print('Destino elegido. Navegando hasta el punto...')

            goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = random_x * map_data.info.resolution + map_data.info.origin.position.x
            goal.target_pose.pose.position.y = random_y * map_data.info.resolution + map_data.info.origin.position.y
            goal.target_pose.pose.orientation.w = 1.0

            goal_client.send_goal(goal, active_cb=None, feedback_cb=lambda feedback: callback_feedback(feedback, goal_client), done_cb=None)
            wait = goal_client.wait_for_result()
            print('Punto alcanzado!')
        else:
            system("rosrun map_server map_saver -f nombreDelMapa1")
            print("MAPA MAPEADO :))))")
            exit(1)

```

El problema que he encontrado durante el desarrollo del script es que el robot checkea puntos que están fuera del mapa en si, por lo que he tenido que realizar una comprobación en la que si la distancia del robot no ha cambiado apenas en dos iteraciones pasa al siguiente punto:

`callback_feedback`
```python
def callback_feedback(feedback, goal_client):

    global aux_x, aux_y

    if truncate_float(aux_x, 4) == truncate_float(float(feedback.base_position.pose.position.x), 4) and truncate_float(aux_y, 4) == truncate_float(float(feedback.base_position.pose.position.y), 4):
        rospy.loginfo("Deteniendo el objetivo...")
        goal_client.cancel_goal()

    aux_x = float(feedback.base_position.pose.position.x)
    aux_y = float(feedback.base_position.pose.position.y)
```
## Resultados algoritmo de exploración
Estos son los resultados que he obtenido: