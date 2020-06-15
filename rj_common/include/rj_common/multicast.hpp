#pragma once

#include <QAbstractSocket>

bool multicast_add(QAbstractSocket* socket, const char* addr);
bool multicast_add_native(int socket, const char* addr);
