# Сервер и клиент для работы GPS приёмника EMLID в RTK режиме

## RTK_server.py
"""
Программа для трансляции поправки с BASE RTK через шириковещательный канал.
Данные принимаются через 2 канала на выбор:<br>
 1. TCP/IP client
 2. UART

**Вывод:**<br>
**ip:** broncast/UDP<br>
**port:** 9000


## RTK_client.py
Программа принимает поправку с РТК через бродкас и отправляет её на GPS приёмник.

**port:** 9000