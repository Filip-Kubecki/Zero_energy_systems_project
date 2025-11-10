# Monitor Wnętrza Obudowy Fotolitografu (Platforma Testowa)

Projekt ten stanowi platformę testową dla systemu monitorowania wnętrza obudowy maszyny fotolitograficznej. Głównym celem jest weryfikacja niskopoziomowej obsługi czujników (odczyty I2C) oraz transmisji zebranych danych przez interfejs UART (jako proof-of-concept).

Projekt jest rozwijany w VS Code przy użyciu wtyczek STM32Cube (CMake).

## Kluczowe Funkcje

* Monitorowanie temperatury (TMP119, ILPS28QSW, HDC3022-Q1)
* Monitorowanie ciśnienia barycznego (ILPS28QSW)
* Monitorowanie wilgotności względnej (HDC3022-Q1)
* Monitorowanie natężenia światła (TCS3720)
* Monitorowanie jakości powietrza (SEN54 - PM, VOC, itp.)
* **Transmisja zebranych danych przez UART** do zewnętrznego modułu komunikacyjnego.

## Struktura Kodu

Kod projektu jest zorganizowany w następujący sposób, aby oddzielić główną logikę aplikacji od sterowników peryferyjnych:

* **`Core/Src/main.c`**: Główny plik aplikacji. Zawiera pętlę `while(1)` oraz logikę koordynującą odczyty z czujników i transmisję danych przez UART.
* **`Drivers/Inc`**: Pliki nagłówkowe (`.h`) dla wszystkich niestandardowych sterowników (np. `HDC3022-Q1.h`, `ILPS28QSW.h`, `TCS3720.h` itd.).
* **`Drivers/Src`**: Pliki źródłowe (`.c`) zawierające implementację funkcji (logikę I2C) dla każdego z czujników.

## Wykorzystany hardware

* **Płytka rozwojowa:** NUCLEO-L152RE
* **Moduł komunikacyjny:** Dowolny zewnętrzny nadajnik bezprzewodowy z interfejsem UART (w tym przypadku ESP32 jako mostek WiFi)
* **Czujniki (I2C):**
    * TMP119 (temperatura)
    * ILPS28QSW (ciśnienie, temperatura)
    * HDC3022-Q1 (wilgotność, temperatura)
    * TCS3720 (natężenie światła)
    * SEN54 (jakość powietrza)

## Kontekst Projektu

* Jest to projekt realizowany w ramach **zajęć uniwersyteckich**.
* Celem jest demonstracja **koncepcji** zbierania i przesyłania danych.
* Projekt **nie jest** niezawodnym urządzeniem badawczym i nie był testowany pod kątem długoterminowej stabilności lub precyzji w warunkach laboratoryjnych.

## Licencja

Ten projekt jest udostępniany na licencji **GNU General Public License v3.0 (GPLv3)**.

Oznacza to, że masz swobodę uruchamiania, studiowania, udostępniania i modyfikowania oprogramowania. Jeśli rozpowszechniasz zmodyfikowane wersje (lub jakiekolwiek dzieła pochodne), muszą one być również udostępniane na licencji GPLv3.

Kod jest dostarczany "jak jest", bez żadnej gwarancji, wyrażonej ani domniemanej. Autor nie ponosi odpowiedzialności za jakiekolwiek błędy, uszkodzenia sprzętu ani inne szkody wynikające z jego użycia.
