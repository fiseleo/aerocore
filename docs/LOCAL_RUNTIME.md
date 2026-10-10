# 本機開發執行資料（0.5）

原始碼與建置產物仍在專案目錄；頻繁存取的遊戲資料使用本機硬碟。
根目錄是 `%USERPROFILE%\.mcmod\runs\aerocore`，每台電腦各自一份：
原開發機為 `C:\Users\ZHU YU TA\.mcmod\runs\aerocore`，`DESKTOP-KLLC9QQ` 為 `C:\Users\CHU YU TA\.mcmod\runs\aerocore`。

| 子目錄 | 內容 |
| --- | --- |
| `client` | 開發客戶端的 saves、options.txt、config、logs、screenshots 等 |
| `server` | 開發專用伺服器的世界與設定 |
| `gametest` | 自動 GameTest 世界，與玩家存檔隔離 |

Gradle 使用 Java 的 `user.home`，不依賴可能被封裝應用程式重新導向的 AppData。
`gradle.properties` 的 `aerocore_runtime_id=aerocore` 使 Z: 與 UNC 別名共用同一個位置。
若另外建立獨立實驗專案，改用另一個識別字；只允許英數字、底線與連字號。
在 IDEA 執行 **Reload All Gradle Projects**，再用 **Minecraft Client** 啟動。

```powershell
.\dev.ps1 :neoforge:showRuntimeDirectories
.\dev.ps1 :neoforge:runClient
```

## 既有資料

原開發機已將 `neoforge/run` 的兩個世界及設定複製到 `client`，78 個檔案逐一驗證 SHA-256。
`session.lock` 不複製，交由 Minecraft 重新建立。根目錄的 `migration-manifest.json` 記錄來源、時間、大小與雜湊。
網路磁碟原檔保留為遷移時的備份，之後的遊戲進度只寫入本機。
已用遷移後的 QA 世界驗證載入、遊玩、儲存並返回主選單；記錄確認全部維度存檔完成，本機 level.dat 更新，網路備份時間未改變。

`scripts/migrate-runtime.ps1` 用於尚未遷移的電腦；先關閉使用來源／目標世界的 Minecraft。
腳本會取得世界鎖、先檢查全部檔案衝突，再複製與驗證。目標若已有不同內容會停止，不覆寫較新的本機進度。
遷移已完成的電腦無須重跑。腳本不合併兩份已各自遊玩的世界。
`DESKTOP-KLLC9QQ` 於 2026-10-07 建立環境時刻意不複製舊備份，避免與原開發機的進度分歧；
需要既有世界時，關閉兩邊遊戲後從原開發機複製 `client/saves`。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\migrate-runtime.ps1
```

這是單次程序的執行設定，無須改全機原則。正常啟動也可使用相同方式：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\dev.ps1 :neoforge:runClient
```

搬到其他電腦或做備份時，關閉遊戲後複製本機 `client/saves`；只備份專案原始碼不會包含之後的遊戲進度。

## 建立世界時崩潰：Unable to establish loopback connection

Windows 上的 Java NIO 會在暫存目錄建立 Unix socket 當作內部管道；單人世界的內建伺服器連線與版本檢查都需要它。
某些環境（封裝應用程式的 TEMP 重新導向、`DESKTOP-KLLC9QQ` 的系統 TEMP）在那裡建立會失敗，錯誤是
`failed to open a new selector … Unable to establish loopback connection … Invalid argument: connect`。

`neoforge/build.gradle` 因此讓所有遊戲 run 使用 `-Djava.io.tmpdir` 與 `-Djdk.net.unixdomain.tmpdir` 指向
`%USERPROFILE%\.mcmod\tmp`，與 `dev.ps1` 相同；不論 IDE 是不是由 `open-ide.cmd` 啟動都有效。
修改建置設定後，在 IDEA 按 **Reload All Gradle Projects** 再啟動 **Minecraft Client**。
