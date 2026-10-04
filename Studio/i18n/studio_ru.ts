<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<!-- Russian UI strings of ManipStudio. Sources are the English strings of
     qsTr()/tr() in the QML files and RobotBackend; contexts are the QML file
     names. Read at run time by TsTranslator; editable with Qt Linguist. -->
<TS version="2.1" language="ru_RU" sourcelanguage="en">
<context>
    <name>Main</name>
    <message><source>loading</source><translation>загрузка</translation></message>
    <message><source>planning</source><translation>расчёт</translation></message>
    <message><source>ready</source><translation>готов</translation></message>
    <message><source>Motors</source><translation>Моторы</translation></message>
    <message><source>Settings</source><translation>Настройки</translation></message>
    <message><source> s</source><translation> с</translation></message>
</context>
<context>
    <name>JointPanel</name>
    <message><source>Joints</source><translation>Суставы</translation></message>
    <message><source>Zero</source><translation>Ноль</translation></message>
    <message><source>Folded</source><translation>Сложенная</translation></message>
</context>
<context>
    <name>MovePanel</name>
    <message><source>Motion</source><translation>Движение</translation></message>
    <message><source>m</source><translation>м</translation></message>
    <message><source>Through point</source><translation>Через точку</translation></message>
    <message><source>End pose</source><translation>Конечная поза</translation></message>
    <message><source>Speed, °/s</source><translation>Скорость, °/с</translation></message>
    <message><source>Speed, m/s</source><translation>Скорость, м/с</translation></message>
    <message><source>Acceleration, °/s²</source><translation>Ускорение, °/с²</translation></message>
    <message><source>Acceleration, m/s²</source><translation>Ускорение, м/с²</translation></message>
    <message><source>Current</source><translation>Текущие</translation></message>
    <message><source>To program</source><translation>В программу</translation></message>
    <message><source>Stop</source><translation>Стоп</translation></message>
    <message><source>Run</source><translation>Выполнить</translation></message>
</context>
<context>
    <name>JogPanel</name>
    <message><source>Tool jog</source><translation>Перемещение инструмента</translation></message>
    <message><source>Step</source><translation>Шаг</translation></message>
    <message><source>mm</source><translation>мм</translation></message>
</context>
<context>
    <name>ProgramPanel</name>
    <message><source>Program</source><translation>Программа</translation></message>
    <message><source>Empty. Set a target in the motion panel and press «To program».</source><translation>Пусто. Задайте цель в панели движения и нажмите «В программу».</translation></message>
    <message><source>Run program</source><translation>Запустить</translation></message>
    <message><source>Save</source><translation>Сохранить</translation></message>
    <message><source>Load</source><translation>Загрузить</translation></message>
    <message><source>Clear</source><translation>Очистить</translation></message>
    <message><source>Save program</source><translation>Сохранить программу</translation></message>
    <message><source>Program (*.json)</source><translation>Программа (*.json)</translation></message>
    <message><source>Load program</source><translation>Загрузить программу</translation></message>
</context>
<context>
    <name>PosePanel</name>
    <message><source>Tool (TCP)</source><translation>Инструмент (TCP)</translation></message>
    <message><source>m</source><translation>м</translation></message>
</context>
<context>
    <name>PeaksPanel</name>
    <message><source>Peak moments</source><translation>Пиковые моменты</translation></message>
    <message><source>no trajectory</source><translation>нет траектории</translation></message>
    <message><source> N·m</source><translation> Н·м</translation></message>
    <message><source>Trajectory rejected: </source><translation>Траектория отклонена: </translation></message>
</context>
<context>
    <name>ResultsPanel</name>
    <message><source>Moments</source><translation>Моменты</translation></message>
    <message><source>Angles</source><translation>Углы</translation></message>
    <message><source>Speeds</source><translation>Скорости</translation></message>
    <message><source>Accelerations</source><translation>Ускорения</translation></message>
    <message><source>N·m</source><translation>Н·м</translation></message>
    <message><source>°/s</source><translation>°/с</translation></message>
    <message><source>°/s²</source><translation>°/с²</translation></message>
    <message><source>limits</source><translation>пределы</translation></message>
    <message><source>t, s</source><translation>t, с</translation></message>
    <message><source>Pause</source><translation>Пауза</translation></message>
    <message><source>Play</source><translation>Пуск</translation></message>
    <message><source> s</source><translation> с</translation></message>
    <message><source>Run a MoveJ to see the trajectory plots here</source><translation>Выполните MoveJ — здесь появятся графики траектории</translation></message>
    <message><source>Save trajectory</source><translation>Сохранить траекторию</translation></message>
    <message><source>CSV (*.csv)</source><translation>CSV (*.csv)</translation></message>
</context>
<context>
    <name>RobotView</name>
    <message><source>Iso</source><translation>Изо</translation></message>
    <message><source>Front</source><translation>Спереди</translation></message>
    <message><source>Side</source><translation>Сбоку</translation></message>
    <message><source>Top</source><translation>Сверху</translation></message>
    <message><source>LMB: rotate · wheel: zoom · RMB / two fingers: pan</source><translation>ЛКМ — вращение · колесо — масштаб · ПКМ / два пальца — сдвиг</translation></message>
</context>
<context>
    <name>RobotBackend</name>
    <message><source>Loading model from %1</source><translation>Загрузка модели из %1</translation></message>
    <message><source>Model loaded, floor at z = %1 m</source><translation>Модель загружена, пол на высоте z = %1 м</translation></message>
    <message><source>%1 failed</source><translation>%1: ошибка</translation></message>
    <message><source>%1: the solver is busy</source><translation>%1: решатель занят</translation></message>
    <message><source>Planning %1...</source><translation>Расчёт %1...</translation></message>
    <message><source>%1 failed after %2 samples</source><translation>%1 прерван после %2 точек</translation></message>
    <message><source>%1: %2 samples, %3 s</source><translation>%1: %2 точек, %3 с</translation></message>
    <message><source>%1: %2 samples, %3 s, slowed to %4% by the q%5 speed limit</source><translation>%1: %2 точек, %3 с, замедлено до %4% из-за предела скорости q%5</translation></message>
    <message><source>%1: %2 samples, %3 s, slowed to %4% by the q%5 acceleration limit</source><translation>%1: %2 точек, %3 с, замедлено до %4% из-за предела ускорения q%5</translation></message>
    <message><source>Program: %1</source><translation>Программа: %1</translation></message>
    <message><source>Program</source><translation>Программа</translation></message>
    <message><source>Program: no steps</source><translation>Программа: нет шагов</translation></message>
    <message><source>Program: cannot open %1</source><translation>Программа: не удаётся открыть %1</translation></message>
    <message><source>Settings: the solver is busy</source><translation>Настройки: решатель занят</translation></message>
    <message><source>Settings: every field needs a number</source><translation>Настройки: в каждом поле должно быть число</translation></message>
    <message><source>Saving settings...</source><translation>Сохранение настроек...</translation></message>
    <message><source>Applying settings...</source><translation>Применение настроек...</translation></message>
    <message><source>Settings saved to %1</source><translation>Настройки сохранены в %1</translation></message>
    <message><source>Settings applied (not saved)</source><translation>Настройки применены (не сохранены)</translation></message>
    <message><source>Settings reloaded from %1</source><translation>Настройки перечитаны из %1</translation></message>
</context>
<context>
    <name>SettingsPanel</name>
    <message><source>Settings</source><translation>Настройки</translation></message>
    <message><source>unsaved changes</source><translation>есть несохранённые изменения</translation></message>
    <message><source>Kinematics</source><translation>Кинематика</translation></message>
    <message><source>Limits</source><translation>Пределы</translation></message>
    <message><source>Dynamics</source><translation>Динамика</translation></message>
    <message><source>Modified Denavit-Hartenberg table: frame i is reached from frame i-1 by α and a of row i-1, then θ and d of row i. The radius sets the joint drum of the 3D and collision model.</source><translation>Модифицированная таблица Денавита–Хартенберга: система i получается из i-1 через α и a строки i-1, затем θ и d строки i. Радиус задаёт барабан сустава в 3D-модели и модели коллизий.</translation></message>
    <message><source>Joint</source><translation>Сустав</translation></message>
    <message><source>a, m</source><translation>a, м</translation></message>
    <message><source>α, °</source><translation>α, °</translation></message>
    <message><source>d, m</source><translation>d, м</translation></message>
    <message><source>Radius, m</source><translation>Радиус, м</translation></message>
    <message><source>min, °</source><translation>мин, °</translation></message>
    <message><source>max, °</source><translation>макс, °</translation></message>
    <message><source>Moment, N·m</source><translation>Момент, Н·м</translation></message>
    <message><source>Speed, °/s</source><translation>Скорость, °/с</translation></message>
    <message><source>Accel., °/s²</source><translation>Ускор., °/с²</translation></message>
    <message><source>MoveJ keeps every joint within its speed and acceleration; MoveL and MoveC slow the whole move down instead of leaving the path.</source><translation>MoveJ держит каждый сустав в пределах его скорости и ускорения; MoveL и MoveC вместо ухода с пути равномерно замедляют всё движение.</translation></message>
    <message><source>Floor height, m</source><translation>Высота пола, м</translation></message>
    <message><source>Payload (point mass in the flange frame)</source><translation>Груз (точечная масса в системе фланца)</translation></message>
    <message><source>Mass, kg</source><translation>Масса, кг</translation></message>
    <message><source>Centre of mass x, y, z, m</source><translation>Центр масс x, y, z, м</translation></message>
    <message><source>Links: mass, kg and centre of mass in the joint frame, m</source><translation>Звенья: масса, кг, и центр масс в системе сустава, м</translation></message>
    <message><source>Link</source><translation>Звено</translation></message>
    <message><source>Inertia about the joint frame origin, kg·m²</source><translation>Инерция относительно начала системы сустава, кг·м²</translation></message>
    <message><source>Apply rebuilds the planner with these values; Save also writes them to %1.</source><translation>«Применить» пересоздаёт планировщик с этими значениями; «Сохранить» ещё и записывает их в %1.</translation></message>
    <message><source>Revert</source><translation>Отменить</translation></message>
    <message><source>Reload file</source><translation>Перечитать файл</translation></message>
    <message><source>Apply</source><translation>Применить</translation></message>
    <message><source>Save to file</source><translation>Сохранить в файл</translation></message>
</context>
<context>
    <name>MotorHub</name>
    <message><source>%1: at the end of its range</source><translation>%1: конец диапазона</translation></message>
    <message><source>%1: calibrate from IDLE only</source><translation>%1: калибровка только из IDLE</translation></message>
    <message><source>%1: calibrating, the motor will beep and turn</source><translation>%1: калибровка, мотор пищит и вращается</translation></message>
    <message><source>%1: closed loop requested</source><translation>%1: включение замкнутого контура</translation></message>
    <message><source>%1: enable the motor first</source><translation>%1: сначала включите мотор</translation></message>
    <message><source>%1: errors cleared</source><translation>%1: ошибки сброшены</translation></message>
    <message><source>%1: idle, the motor is free</source><translation>%1: IDLE, мотор свободен</translation></message>
    <message><source>%1: invalid motor settings</source><translation>%1: неверные настройки мотора</translation></message>
    <message><source>%1: motor error %2</source><translation>%1: ошибка мотора %2</translation></message>
    <message><source>%1: motor online (node %2)</source><translation>%1: мотор на связи (узел %2)</translation></message>
    <message><source>%1: no answer from node %2</source><translation>%1: нет ответа от узла %2</translation></message>
    <message><source>%1: no position from the motor yet</source><translation>%1: позиция от мотора ещё не пришла</translation></message>
    <message><source>%1: target limited to %2°</source><translation>%1: цель ограничена до %2°</translation></message>
    <message><source>%1: the motor is offline</source><translation>%1: мотор не на связи</translation></message>
    <message><source>%1: zero set at %2 motor turns</source><translation>%1: ноль установлен на %2 об. мотора</translation></message>
    <message><source>All motors idle</source><translation>Все моторы в IDLE</translation></message>
    <message><source>CAN error: %1</source><translation>Ошибка CAN: %1</translation></message>
    <message><source>Cannot connect: %1</source><translation>Не удалось подключиться: %1</translation></message>
    <message><source>Cannot write %1</source><translation>Не удалось записать %1</translation></message>
    <message><source>Connected to %1, waiting for the axes</source><translation>Подключено к %1, ждём оси</translation></message>
    <message><source>Motor settings saved to %1</source><translation>Настройки моторов сохранены в %1</translation></message>
    <message><source>Not connected</source><translation>Не подключено</translation></message>
    <message><source>Stop: holding position</source><translation>Стоп: удержание позиции</translation></message>
</context>
<context>
    <name>MotorWindow</name>
    <message><source>Accel., °/s²</source><translation>Ускор., °/с²</translation></message>
    <message><source>Axes</source><translation>Оси</translation></message>
    <message><source>CAN node</source><translation>Узел CAN</translation></message>
    <message><source>Calibrate %1?</source><translation>Калибровать %1?</translation></message>
    <message><source>Calibrate</source><translation>Калибровка</translation></message>
    <message><source>Clear errors</source><translation>Сбросить ошибки</translation></message>
    <message><source>Connect</source><translation>Подключить</translation></message>
    <message><source>Current</source><translation>Ток</translation></message>
    <message><source>Current, A</source><translation>Ток, А</translation></message>
    <message><source>Direction</source><translation>Направление</translation></message>
    <message><source>Disable all</source><translation>Отключить все</translation></message>
    <message><source>Disable</source><translation>Отключить</translation></message>
    <message><source>Disconnect</source><translation>Отключиться</translation></message>
    <message><source>Enable</source><translation>Включить</translation></message>
    <message><source>Esc = stop</source><translation>Esc = стоп</translation></message>
    <message><source>Gear ratio</source><translation>Передаточное число</translation></message>
    <message><source>Go to, °</source><translation>В угол, °</translation></message>
    <message><source>Go</source><translation>Пуск</translation></message>
    <message><source>Here</source><translation>Здесь</translation></message>
    <message><source>Jog</source><translation>Толчок</translation></message>
    <message><source>Joint angle = direction × (motor turns − zero) / gear × 360°. Set the ODrive watchdog: the motors keep the last command if this program stops talking.</source><translation>Угол сустава = направление × (обороты мотора − ноль) / передаточное × 360°. Включите watchdog в ODrive: без него моторы продолжают последнюю команду, если программа замолчит.</translation></message>
    <message><source>Live</source><translation>Онлайн</translation></message>
    <message><source>Motors</source><translation>Моторы</translation></message>
    <message><source>Position</source><translation>Позиция</translation></message>
    <message><source>Range, °</source><translation>Диапазон, °</translation></message>
    <message><source>STOP</source><translation>СТОП</translation></message>
    <message><source>Save settings</source><translation>Сохранить настройки</translation></message>
    <message><source>Settings</source><translation>Настройки</translation></message>
    <message><source>Speed</source><translation>Скорость</translation></message>
    <message><source>Speed, °/s</source><translation>Скорость, °/с</translation></message>
    <message><source>Test</source><translation>Тест</translation></message>
    <message><source>The motor will beep and turn by itself for several seconds. The joint must be free to move and nothing may be in its way.</source><translation>Мотор будет пищать и сам вращаться несколько секунд. Сустав должен свободно двигаться, на пути ничего не должно быть.</translation></message>
    <message><source>Zero, turns</source><translation>Ноль, об.</translation></message>
    <message><source>not connected</source><translation>не подключено</translation></message>
    <message><source>offline</source><translation>нет связи</translation></message>
</context>
</TS>
