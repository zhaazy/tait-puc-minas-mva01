# Analisador de Ventilação Mecânica

> Desenvolvimento de um dispositivo acessível para aferição e calibração de ventiladores mecânicos.


## 📖 Sobre o Projeto

Este projeto consiste no desenvolvimento de um analisador de ventiladores mecânicos portátil e de baixo custo. O objetivo é fornecer uma ferramenta confiável para a verificação de parâmetros críticos de ventilação (como fluxo e pressão), auxiliando na manutenção hospitalar e garantindo a segurança dos equipamentos de suporte à vida.

O sistema é composto por um dispositivo físico (baseado no ESP32) que realiza a aquisição de dados e os exibe localmente em um display TFT, além de transmiti-los via Bluetooth para um aplicativo Android dedicado.

## ⚙️ Principais Funcionalidades

* **Monitoramento em Tempo Real:** Visualização de gráficos de pressão e fluxo.
* **Conectividade Bluetooth:** Transmissão de dados sem fio para dispositivos móveis.
* **Interface Local:** Display TFT colorido para feedback imediato no dispositivo.
* **Alta Precisão:** Utilização de conversor Analógico-Digital (ADC) externo de 16-bits.
* **Portabilidade:** Design compacto e alimentado por bateria (se aplicável).

## 🛠️ Hardware e Tecnologias

### Componentes Principais
* **Microcontrolador:** ESP32 (Wi-Fi/Bluetooth).
* **Display:** TFT SPI 240x320 pixels.
* **Aquisição de Sinais:** Módulo ADS1115 (ADC de 16-bits) para leitura precisa dos sensores.
* **Sensores:** (Liste aqui seus sensores de pressão/fluxo específicos).

### Software
* **Firmware:** C++ / Framework Arduino (ou ESP-IDF).
