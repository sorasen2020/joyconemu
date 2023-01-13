# joyconemu
esp32でJoycon(L/R)/Pro Controllerをエミュレートし、PokeControllerで動かす

## 使いかた
esp-idfインストール済の環境で以下のコマンド実行
```
cd joyconemu
idf.py build
idf.py -p (PORT) flash
```
## ざっくりとした動かし方
https://note.com/rokkoku/n/nd6a57286ec13

pressはduration=0.15, wait=0.15以上確保し、Show Serialにチェックを入れることを推奨


以下のコンフィグでピカブイを動かせることを確認

#define CONTROLLER_TYPE (JOYCON_L)

## 追記(2023.01.13)

### コンパイル環境

- Visual Studio Code + Espressif IDF(拡張)
- esp-idf version release/v5.0

参考URL:https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md

### 注意

- COMポート番号、Flashサイズ、チップ名は各環境に合わせて要修正
- ログの出力をOFFにしてるので、デバッグする際はLog outputのDefault log verbosityをNo Outputから変更すること
- C/C++構成はWin32(Release)でコンパイル

