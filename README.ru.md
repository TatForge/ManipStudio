<div align="center">

# ManipStudio

**Настройте робота. Спланируйте движение. Посмотрите нагрузки.**

![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)
![Qt](https://img.shields.io/badge/Qt-6.4%2B-41CD52?style=flat-square)
![Платформы](https://img.shields.io/badge/platforms-Windows%20%7C%20Linux-667085?style=flat-square)

[English](README.md)

</div>

![ManipStudio: симуляция робота, программа движений и графики моментов](docs/images/manipstudio.png)

Приложение для моделирования и симуляции шестикоординатных манипуляторов.
Настройка робота, планирование траекторий и анализ нагрузок — в одном окне.

## Возможности

- Интерактивная 3D-модель, путь TCP и воспроизведение по шкале времени.
- MoveJ, MoveL, MoveC, перемещение TCP и программы движений в JSON.
- Настройка DH-геометрии, пределов суставов, динамики звеньев и груза.
- Проверка столкновений, ограничений скоростей, ускорений и моментов.
- Графики движения и моментов с экспортом в CSV.
- Русский и английский интерфейс.

## Сборка и запуск

Нужны компилятор C++17, CMake, Ninja, Git и Qt 6.4+ с модулями Quick, Quick Controls, Quick3D, Charts и Test.
Для Windows используйте окружение MSYS2 UCRT64.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
./build/Studio/ManipStudio
```

В Windows запускайте `./build/Studio/ManipStudio.exe`.
Настройки доступны в приложении и сохраняются в `ManipulatorSettigns.ini`.
Пример программы движений: [Studio/examples/demo_program.json](Studio/examples/demo_program.json).

## Тесты

```bash
ctest --test-dir build --output-on-failure
```
