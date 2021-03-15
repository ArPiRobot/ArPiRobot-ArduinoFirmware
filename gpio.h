#pragma once

#include <stdint.h>

void doPinMode(const char *args);

void doDigWrite(const char *args);

void doDigRead(const char *args);

void doAnaRead(const char *args);

void doAnaToDig(const char *args);

void doAnaWrite(const char *args);

// TODO
void startAutoDigRead(const char *args);

void stopAutoDigRead(const char *args);

void startAutoAnaRead(const char *args);

void stopAutoAnaRead(const char *args);

void startDigCount(const char *args);

void stopDigCount(const char *args);
