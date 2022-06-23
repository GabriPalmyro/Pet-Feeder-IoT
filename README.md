# 😺💡 Pet-Feeder-Iot

O projeto consiste em um alimentador de pet IOT que possui dois modos de acionamento parao despejar a comida do animal: o primeiro modo é através de um sensor de presença (PIR), quando detectada a presença em uma área próxima ao alimentador, um servo motor é acionado girando a estrutura que armazena a comida e consequentemente despejando-a em um pote; o segundo modo de acionamento é feito por um aplicativo que está conectado via *protocolo MQTT* a placa de desenvolvimento ESP8266 que será responsável por acionar o servo motor quando o usuário desejar.
O aplicativo também possui funções a mais para monitoramento e controle do alimentador, é possível reproduzir um aviso sonoro ao animal através de um buzzer para que ele seja atraído ao alimentador, monitorar o nível de água de um recipiente com um sensor de nível de água e monitorar se a presença está sendo detectada ou não.

## 📷 Fotos do projeto<table>

<table>
  <tr>
    <th><img src="https://github.com/GabriPalmyro/Pet-Feeder-IoT/blob/main/images/circuito-pet-feeder.png?raw=true" width=80% > <br> <sub> Circuito Esquemático </sub></th>
    <th><img src="https://github.com/GabriPalmyro/Pet-Feeder-IoT/blob/main/images/prototipo1.jpg?raw=true" width=150% > <br> <sub> Protótipo </sub></th>
  </tr>
</table>


## ⚒️ Funcionalidades
* Controle a distância do acionamento do alimentador através do aplicativo
* Uso de sensores como o PIR (Passive Infra Red) e o sensor de nível de água
* Monitoramento do nível de água em tempo real

<!-- ROADMAP -->
## 🌌 Roadmap

- [ ] Controle de Wi-Fi conectado ao ESP8266
- [ ] Setar a duração da queda do alimentador

## 🔧 Feito Com

* [ESP8266](https://google.com/search?q=esp8266)
* [MQTT PROTOCOL](https://pt.wikipedia.org/wiki/MQTT)
* [ADAFRUIT IO](https://io.adafruit.com/)
* [MQTT DASHBOARD APP](https://play.google.com/store/apps/details?id=com.app.vetru.mqttdashboard&hl=pt_BR&gl=US)

## 👨🏽‍💻 Desenvolvedores

<table>
  <tr>
    <th><img src="https://avatars.githubusercontent.com/u/62028766?v=4" width=115 > <br> <sub> Gabriel Palmyro Martins </sub></th>
    <th><img src="https://avatars.githubusercontent.com/u/57163905?v=4" width=115 > <br> <sub> Pedro Henrique Ton Pauletti </sub></th>
  </tr>
</table>
