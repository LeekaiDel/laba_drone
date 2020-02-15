# Файлы для настйроки pixhawk

## Содержание
1. FMU - прошивки для pixhawk и px4flow
2. params - параметры настройки для разный режимов работы
3. rules - файл с настройками портов
4. etc - файл с настройками скорсоти телеметрии, необходимо поместить на флешку pixhawk


Из папки **rules** выполнить

```
sudo cp drone-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
