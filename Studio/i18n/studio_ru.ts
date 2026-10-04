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
</TS>
