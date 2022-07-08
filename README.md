# joyconemu
esp32でJoycon(R)をエミュレートし、PokeControllerで動かす

## 使いかた
esp-idfインストール済の環境で以下のコマンド実行
```
cd joyconemu
idf.py build
idf.py -p (PORT) flash
```
## ざっくりとした動かし方
https://note.com/rokkoku/n/nd6a57286ec13

pressはduration=0.15, wait=0.15以上確保することを推奨


以下のコンフィグでピカブイを動かせることを確認

#define CONTROLLER_TYPE (JOYCON_L)
