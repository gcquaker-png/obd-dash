#pragma once
// FW_VER      — семантическая версия из файла VERSION (правится вручную при релизе)
// FW_REV      — короткий git-хеш (подставляется при сборке: flash.sh / CI)
// FW_VERSION  — полная строка для показа: "1.0.4 (abc1234)"
#ifndef FW_VER
#define FW_VER "0.0.0"
#endif
#ifndef FW_REV
#define FW_REV "dev"
#endif
#define FW_VERSION FW_VER " (" FW_REV ")"
#ifndef FW_BUILD_DATE
#define FW_BUILD_DATE __DATE__
#endif
