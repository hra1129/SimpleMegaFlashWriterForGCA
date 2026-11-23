// =============================================================================
// SMFWriter - Simple Mega Flash Cartridge Writer (Windows コマンドライン版)
//
// MSXPLAYer Game Cartridge Adapter (USB CDC 仮想 COM ポート) を経由して、
// Simple Mega Flash カートリッジへ ROM イメージを書き込むツール。
//
// カートリッジ側の仕様 (MSX_SimpleMegaFlash リポジトリより):
//   - Flash ROM : Microchip SST39SF040A (4Mbit = 512KB)
//                 Maker ID = 0xBF, Device ID = 0xB7 (010A=0xB5 / 020A=0xB6)
//   - Flash コマンドアドレスは CPLD が固定デコードする:
//       スロット 0x3555 -> Flash 0x5555
//       スロット 0x2AAA -> Flash 0x2AAA
//   - バイトプログラム手順 (同梱 SW.COM と同一):
//       バンクセレクト -> AA@3555 -> 55@2AAA -> A0@3555 -> データ書込
//       -> バンク再セレクト -> 読み戻しが一致するまでポーリング
//   - チップ消去: AA,55,80 / AA,55,10 (上記アドレス使用)
//
// アダプタ側の仕様 (MSXPLAYer_GameAdapter リポジトリより):
//   - テキストコマンド (カンマ区切り / 引数は16進 / CRLF 終端)、応答最終行は OK / FAIL
//   - スクリプトエンジン (BSCR) でバイトプログラム+ポーリングをデバイス側実行する
//     (公式サンプル MSXCR_Simple64kWrite と同じ方式)
//
// ビルド: Visual Studio 2022 (v143), Win32 API のみ使用 (setupapi.lib をリンク)
// =============================================================================

#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <setupapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdint>
#include <cwctype>
#include <string>
#include <vector>

#pragma comment(lib, "setupapi.lib")

// -----------------------------------------------------------------------------
// ユーティリティ
// -----------------------------------------------------------------------------

// ワイド文字列を UTF-8 文字列へ (コンソール出力用)
// ※ 出力は printf + UTF-8 ナロー文字列に統一している。wprintf は "C" ロケールで
//   変換するため、日本語に当たった時点でその文字列の出力が打ち切られてしまう。
static std::string ToUtf8(const std::wstring& w)
{
    if (w.empty()) return std::string();
    int len = WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), NULL, 0, NULL, NULL);
    if (len <= 0) return std::string();
    std::string s((size_t)len, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), &s[0], len, NULL, NULL);
    return s;
}
static std::string ToUtf8(const wchar_t* w)
{
    return (w == NULL) ? std::string() : ToUtf8(std::wstring(w));
}

// 文字列中から "xxxx : yy" 形式 (SMRD 応答) を探して読み値を返す
static bool ParseSmrdValue(const std::string& response, uint16_t expectAddr, uint8_t& value)
{
    size_t pos = 0;
    while (pos < response.size())
    {
        size_t eol = response.find('\n', pos);
        if (eol == std::string::npos) eol = response.size();
        std::string line = response.substr(pos, eol - pos);
        pos = eol + 1;

        // "4000 : bf" 形式かチェック
        unsigned int a = 0, v = 0;
        char extra = 0;
        if (sscanf(line.c_str(), "%4x : %2x %c", &a, &v, &extra) == 2 && a == expectAddr)
        {
            value = (uint8_t)v;
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// シリアル通信 (USB CDC)
// -----------------------------------------------------------------------------
class Serial
{
public:
    Serial() : m_h(INVALID_HANDLE_VALUE) {}
    ~Serial() { Close(); }

    bool Open(const wchar_t* portName)
    {
        Close();
        std::wstring path = L"\\\\.\\";
        path += portName;
        m_h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (m_h == INVALID_HANDLE_VALUE) return false;

        if (!SetupComm(m_h, 65536, 65536))
        {
            Close();
            return false;
        }

        DCB dcb = { sizeof(DCB) };
        if (!GetCommState(m_h, &dcb))
        {
            Close();
            return false;
        }
        // 公式サンプルと同じ設定 (115200, 8-N-1, フロー制御なし)
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fNull = FALSE;
        dcb.fErrorChar = FALSE;
        if (!SetCommState(m_h, &dcb))
        {
            Close();
            return false;
        }

        COMMTIMEOUTS timeout = {};
        timeout.ReadIntervalTimeout = 10;
        timeout.ReadTotalTimeoutMultiplier = 0;
        timeout.ReadTotalTimeoutConstant = 10;
        timeout.WriteTotalTimeoutMultiplier = 0;
        timeout.WriteTotalTimeoutConstant = 3000;
        SetCommTimeouts(m_h, &timeout);

        PurgeComm(m_h, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
        m_rx.clear();
        return true;
    }

    void Close()
    {
        if (m_h != INVALID_HANDLE_VALUE)
        {
            CloseHandle(m_h);
            m_h = INVALID_HANDLE_VALUE;
        }
    }

    bool IsOpen() const { return m_h != INVALID_HANDLE_VALUE; }

    // コマンド文字列だけを送信する (CRLF 付加)
    bool SendRaw(const char* cmd)
    {
        std::string line = cmd;
        line += "\r\n";
        return WriteAll(line.data(), line.size());
    }

    // OK / FAIL の最終行まで受信する。受信した全行を response に返す。
    bool WaitResult(std::string& response, DWORD timeoutMs)
    {
        response.clear();
        ULONGLONG deadline = GetTickCount64() + timeoutMs;
        std::string line;
        for (;;)
        {
            // ループ先頭でも期限を確認する。空行などが受信バッファに残っていると
            // ReadLine が即座に成功し続けるため、ここで確認しないと期限切れ後も
            // バッファを消費し続けて抜けられないことがある。
            if (GetTickCount64() >= deadline) return false;
            if (!ReadLine(line, RemainMs(deadline))) return false;
            if (line.empty()) continue;
            response += line;
            response += "\n";
            if (line == "OK") return true;
            if (line == "FAIL") return false;
        }
    }

    // コマンド送信 + 結果待ち
    bool Command(const char* cmd, std::string& response, DWORD timeoutMs = 10000)
    {
        if (!SendRaw(cmd)) return false;
        return WaitResult(response, timeoutMs);
    }
    bool Command(const std::string& cmd, std::string& response, DWORD timeoutMs = 10000)
    {
        return Command(cmd.c_str(), response, timeoutMs);
    }

    // バイナリ送信 (BRCV 用)
    bool WriteBinary(const uint8_t* data, size_t len)
    {
        return WriteAll(data, len);
    }

    // バイナリを正確に len バイト受信 (BSND 用)
    bool ReadBinary(uint8_t* buf, size_t len, DWORD timeoutMs)
    {
        ULONGLONG deadline = GetTickCount64() + timeoutMs;
        size_t got = 0;
        while (got < len)
        {
            if (!m_rx.empty())
            {
                size_t take = (len - got < m_rx.size()) ? (len - got) : m_rx.size();
                memcpy(buf + got, m_rx.data(), take);
                m_rx.erase(m_rx.begin(), m_rx.begin() + (ptrdiff_t)take);
                got += take;
                continue;
            }
            if (GetTickCount64() >= deadline) return false;
            uint8_t tmp[8192];
            DWORD read = 0;
            if (!ReadFile(m_h, tmp, sizeof(tmp), &read, NULL)) return false;
            if (read > 0) m_rx.insert(m_rx.end(), tmp, tmp + read);
        }
        return true;
    }

    // SMWR: 1バイト書込
    bool WriteByte(uint16_t addr, uint8_t data, DWORD timeoutMs = 10000)
    {
        char cmd[32];
        sprintf(cmd, "SMWR,%04X,%02X", addr, data);
        std::string resp;
        return Command(cmd, resp, timeoutMs);
    }

    // SMRD: 1バイト読込
    bool ReadByte(uint16_t addr, uint8_t& value, DWORD timeoutMs = 10000)
    {
        char cmd[32];
        sprintf(cmd, "SMRD,%04X", addr);
        std::string resp;
        if (!Command(cmd, resp, timeoutMs)) return false;
        return ParseSmrdValue(resp, addr, value);
    }

private:
    static DWORD RemainMs(ULONGLONG deadline)
    {
        ULONGLONG now = GetTickCount64();
        if (now >= deadline) return 0;
        ULONGLONG d = deadline - now;
        return (d > 0x7FFFFFFF) ? 0x7FFFFFFF : (DWORD)d;
    }

    bool WriteAll(const void* data, size_t len)
    {
        const uint8_t* p = (const uint8_t*)data;
        size_t sent = 0;
        while (sent < len)
        {
            DWORD written = 0;
            if (!WriteFile(m_h, p + sent, (DWORD)(len - sent), &written, NULL)) return false;
            if (written == 0) return false;  // 進捗なしで成功を返し続けるケースでの無限ループ防止
            sent += written;
        }
        FlushFileBuffers(m_h);
        return true;
    }

    bool FillOnce()
    {
        uint8_t tmp[4096];
        DWORD read = 0;
        if (!ReadFile(m_h, tmp, sizeof(tmp), &read, NULL)) return false;
        if (read > 0) m_rx.insert(m_rx.end(), tmp, tmp + read);
        return true;
    }

    bool ReadLine(std::string& line, DWORD timeoutMs)
    {
        ULONGLONG deadline = GetTickCount64() + timeoutMs;
        for (;;)
        {
            for (size_t i = 0; i < m_rx.size(); i++)
            {
                if (m_rx[i] == '\n')
                {
                    line.assign((const char*)m_rx.data(), i);
                    m_rx.erase(m_rx.begin(), m_rx.begin() + (ptrdiff_t)i + 1);
                    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
                    return true;
                }
            }
            if (GetTickCount64() >= deadline) return false;
            if (!FillOnce()) return false;
        }
    }

    HANDLE m_h;
    std::vector<uint8_t> m_rx;
};

// -----------------------------------------------------------------------------
// COM ポート列挙とアダプタ検出
// -----------------------------------------------------------------------------

// 列挙した COM ポート1件分 (検出の優先順位とフレンドリ名付き)
struct ComPortInfo
{
    std::wstring name;              // "COM7" など
    std::wstring friendlyName;      // デバイスのフレンドリ名 (無ければ空、表示用)
    int priority;                   // 0: USB VID/PID がアダプタと一致
                                    // 1: その他の USB ポート (フォールバック)
};

// Game Cartridge Adapter の USB VID/PID (ファームウェア usb_descriptors.c より)
//   VID = 2889 (0x0B49), PID = 9938 (0x26D2)
static const wchar_t ADAPTER_USB_ID[] = L"VID_0B49&PID_26D2";

// 大文字小文字を区別せず部分文字列を含むか (ワイド文字列)
static bool ContainsIgnoreCase(const std::wstring& haystack, const wchar_t* needle)
{
    std::wstring h = haystack, n = needle;
    for (auto& c : h) c = towupper(c);
    for (auto& c : n) c = towupper(c);
    return h.find(n) != std::wstring::npos;
}

// ワイド文字列が prefix で始まるか (大文字小文字を区別しない)
static bool StartsWithIgnoreCase(const std::wstring& s, const wchar_t* prefix)
{
    size_t n = wcslen(prefix);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; i++)
        if (towupper(s[i]) != towupper(prefix[i])) return false;
    return true;
}

// デバイスの文字列プロパティ (REG_SZ) を取得する。取れなければ空文字列。
static std::wstring GetDeviceStringProperty(HDEVINFO hDevInfo, SP_DEVINFO_DATA* data, DWORD prop)
{
    wchar_t buf[512] = {};
    if (!SetupDiGetDeviceRegistryPropertyW(hDevInfo, data, prop, NULL,
                                           (PBYTE)buf, sizeof(buf), NULL))
        return std::wstring();
    return std::wstring(buf);
}

// SetupAPI で COM ポートを列挙する (公式サンプルと同じ方式)
// ハードウェアIDが "USB\" で始まるデバイスだけを対象にする。Bluetooth (BTHENUM) 等の
// 仮想 COM ポートは CreateFile が相手への接続試行で長時間ブロックしうるため、
// 列挙の時点で除外して絶対に開かない (公式サンプル MSXCR_Simple64kWrite も同じ方式)。
// あわせて、VID/PID がアダプタと一致するポートに優先の印を付ける。
// フレンドリ名は肯定的な識別には使わない: 実機ではアダプタのフレンドリ名は製品
// 文字列ではなく汎用の「USB シリアル デバイス」であり、OS や言語で変わりうるため。
// フレンドリ名の用途は、"Bluetooth" を含むポートの除外 (USB\ フィルタの補助) と
// 進捗表示だけとする。
static std::vector<ComPortInfo> EnumerateComPorts()
{
    std::vector<ComPortInfo> ports;
    HDEVINFO hDevInfo = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_COMPORT, NULL, NULL,
                                              DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) return ports;

    SP_DEVINFO_DATA data = { sizeof(data) };
    for (DWORD count = 0; SetupDiEnumDeviceInfo(hDevInfo, count, &data); count++)
    {
        wchar_t hwId[512] = {};
        DWORD hwIdBytes = 0;
        if (!SetupDiGetDeviceRegistryPropertyW(hDevInfo, &data, SPDRP_HARDWAREID, NULL,
                                               (PBYTE)hwId, sizeof(hwId), &hwIdBytes))
            continue;
        std::wstring hw((const wchar_t*)hwId, (const wchar_t*)hwId + hwIdBytes / sizeof(wchar_t));
        if (!StartsWithIgnoreCase(hw, L"USB\\")) continue;  // USB デバイス以外は開かない

        HKEY key = SetupDiOpenDevRegKey(hDevInfo, &data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
        if (!key) continue;
        wchar_t portName[256] = {};
        DWORD size = sizeof(portName);
        if (RegQueryValueExW(key, L"PortName", NULL, NULL, (LPBYTE)portName, &size) == ERROR_SUCCESS)
        {
            if (wcsncmp(portName, L"COM", 3) == 0)
            {
                ComPortInfo info;
                info.name = portName;
                info.friendlyName = GetDeviceStringProperty(hDevInfo, &data, SPDRP_FRIENDLYNAME);
                if (info.friendlyName.empty())
                    info.friendlyName = GetDeviceStringProperty(hDevInfo, &data, SPDRP_DEVICEDESC);
                // 名前に Bluetooth を含むポートは開かない (USB\ フィルタの補助)
                if (ContainsIgnoreCase(info.friendlyName, L"Bluetooth"))
                {
                    RegCloseKey(key);
                    continue;
                }
                info.priority = ContainsIgnoreCase(hw, ADAPTER_USB_ID) ? 0 : 1;
                ports.push_back(info);
            }
        }
        RegCloseKey(key);
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
    return ports;
}

// 指定ポートで HVER ハンドシェイクを試み、アダプタなら応答を返す
// アダプタの応答は有線 USB CDC で高速なため、応答待ちは 250ms を1回だけとする
// (アダプタでないポートも 250ms で見切るので、検出全体が速く終わる)。
static bool ProbeAdapter(const wchar_t* portName, std::string& hverResponse)
{
    Serial ser;
    if (!ser.Open(portName)) return false;
    std::string resp;
    if (ser.Command("HVER", resp, 250) &&
        resp.find("FIRMWARE VER") != std::string::npos)
    {
        hverResponse = resp;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// マッパー定義
// -----------------------------------------------------------------------------
struct MapperDef
{
    const char* optName;      // コマンドライン表示名
    const char* fullName;     // 表示用フルネーム
    uint32_t bankSize;        // 1バンクのバイト数
    uint32_t maxRomSize;      // このカートリッジで扱える最大 ROM サイズ
    uint16_t regAddr;         // バンクセレクトレジスタへの書込アドレス
    uint16_t winBase;         // 書込/読込ウィンドウ先頭アドレス
};

static const MapperDef MAPPER_ASCII8   = { "A8",  "ASCII8",   8192,  512 * 1024, 0x7800, 0xA000 };
static const MapperDef MAPPER_ASCII16  = { "A16", "ASCII16",  16384, 512 * 1024, 0x7000, 0x8000 };
static const MapperDef MAPPER_KONAMI8  = { "K8",  "KONAMI8K", 8192,  128 * 1024, 0xA000, 0xA000 };

// Flash コマンドアドレス (CPLD が固定デコードするスロットアドレス)
static const uint16_t FLASH_CMD_5555 = 0x3555;  // Flash の 0x5555 に対応
static const uint16_t FLASH_CMD_2AAA = 0x2AAA;  // Flash の 0x2AAA に対応

// -----------------------------------------------------------------------------
// Flash 操作
// -----------------------------------------------------------------------------

// JEDEC アンロック (AA @ 5555, 55 @ 2AAA)
static bool FlashUnlock(Serial& ser)
{
    return ser.WriteByte(FLASH_CMD_5555, 0xAA) && ser.WriteByte(FLASH_CMD_2AAA, 0x55);
}

// ソフトウェア ID を読む。成功したら maker / device を返す。
static bool FlashReadId(Serial& ser, uint8_t& maker, uint8_t& device)
{
    if (!FlashUnlock(ser)) return false;
    if (!ser.WriteByte(FLASH_CMD_5555, 0x90)) return false;
    bool ok = ser.ReadByte(0x4000, maker) && ser.ReadByte(0x4001, device);
    // ID モード解除 (AA, 55, F0)
    FlashUnlock(ser);
    ser.WriteByte(FLASH_CMD_5555, 0xF0);
    return ok;
}

// チップ消去。完了はアドレス 0x4000 の読み戻し (0xFF) でポーリング確認する。
static bool FlashChipErase(Serial& ser)
{
    if (!FlashUnlock(ser)) return false;
    if (!ser.WriteByte(FLASH_CMD_5555, 0x80)) return false;
    if (!FlashUnlock(ser)) return false;
    if (!ser.WriteByte(FLASH_CMD_5555, 0x10)) return false;

    ULONGLONG deadline = GetTickCount64() + 60000;
    while (GetTickCount64() < deadline)
    {
        uint8_t v = 0;
        if (ser.ReadByte(0x4000, v) && v == 0xFF) return true;
        Sleep(50);
    }
    return false;
}

// スクリプト命令を追加する: [cmd][addr_hi][addr_lo][data]
static void ScriptPush(std::vector<uint8_t>& script, uint8_t cmd, uint16_t addr, uint8_t data)
{
    script.push_back(cmd);
    script.push_back((uint8_t)(addr >> 8));
    script.push_back((uint8_t)(addr & 0xFF));
    script.push_back(data);
}

// 1バイト分のプログラムシーケンスをスクリプトに追加する (SW.COM と同一手順)
static void ScriptProgramByte(std::vector<uint8_t>& script, const MapperDef& mapper,
                              uint8_t segment, uint16_t targetAddr, uint8_t data)
{
    ScriptPush(script, 0x02, mapper.regAddr, segment);        // バンクセレクト
    ScriptPush(script, 0x02, FLASH_CMD_5555, 0xAA);           // アンロック 1
    ScriptPush(script, 0x02, FLASH_CMD_2AAA, 0x55);           // アンロック 2
    ScriptPush(script, 0x02, FLASH_CMD_5555, 0xA0);           // バイトプログラム指令
    ScriptPush(script, 0x02, targetAddr, data);               // データ書込
    ScriptPush(script, 0x02, mapper.regAddr, segment);        // バンク再セレクト
    ScriptPush(script, 0x01, targetAddr, 0x00);               // 読み戻し
    ScriptPush(script, 0x06, targetAddr, data);               // 一致すれば次をスキップ
    ScriptPush(script, 0x0A, 0x0000, 0xFE);                   // JMP -2 (読み戻しへ)
}

// バンク内チャンク分のスクリプトを生成する。0xFF のバイトは消去状態と同じなので除く。
// 戻り値: スクリプトに含めたバイト数 (0 の場合は全て 0xFF)
static size_t BuildChunkScript(std::vector<uint8_t>& script, const MapperDef& mapper,
                               uint8_t segment, const uint8_t* data, size_t len, uint16_t chunkBaseAddr)
{
    script.clear();
    size_t programmed = 0;
    for (size_t i = 0; i < len; i++)
    {
        if (data[i] == 0xFF) continue;
        ScriptProgramByte(script, mapper, segment, (uint16_t)(chunkBaseAddr + i), data[i]);
        programmed++;
    }
    ScriptPush(script, 0xFF, 0x0000, 0x00);  // End
    return programmed;
}

// -----------------------------------------------------------------------------
// メイン処理
// -----------------------------------------------------------------------------

static void PrintUsage(const wchar_t* exeName)
{
    printf("使い方: %s <ROMファイル> /A8|/A16|/K8 [オプション]\n", ToUtf8(exeName).c_str());
    printf("\n");
    printf("  /A8        マッパー: ASCII8   (最大 512KB)\n");
    printf("  /A16       マッパー: ASCII16  (最大 512KB)\n");
    printf("  /K8        マッパー: KONAMI8K (最大 128KB)\n");
    printf("  /P:COMx    アダプタの COM ポートを指定 (省略時は自動検出)\n");
    printf("  /S:n       スロット番号 1 または 2 (既定: 1)\n");
    printf("  /Y         消去前の確認プロンプトを省略\n");
    printf("  /NV        書き込み後のベリファイを省略\n");
    printf("\n");
    printf("前提: カートリッジの SW2 は指定マッパーの位置、SW1 は 4Mbit 位置に\n");
    printf("      設定しておいてください (ツール側では検出・制御しません)。\n");
}

// stdin から1行読む (確認プロンプト用)
static bool ReadStdinLine(std::wstring& line)
{
    wchar_t buf[64];
    if (!fgetws(buf, (int)(sizeof(buf) / sizeof(buf[0])), stdin)) return false;
    line = buf;
    while (!line.empty() && (line.back() == L'\n' || line.back() == L'\r')) line.pop_back();
    return true;
}

int wmain(int argc, wchar_t* argv[])
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    printf("SMFWriter - Simple Mega Flash Writer (Game Cartridge Adapter 経由)\n\n");

    // ---- 引数解析 -----------------------------------------------------------
    std::wstring romPath;
    const MapperDef* mapper = NULL;
    std::wstring optPort;
    int optSlot = 1;
    bool optYes = false;
    bool optNoVerify = false;

    for (int i = 1; i < argc; i++)
    {
        std::wstring arg = argv[i];
        if (arg.empty()) continue;
        if (arg[0] == L'/' || arg[0] == L'-')
        {
            std::wstring opt = arg.substr(1);
            // 大文字化して比較
            for (auto& c : opt) c = (wchar_t)towupper(c);
            if (opt == L"A8" || opt == L"A16" || opt == L"K8")
            {
                if (mapper != NULL)
                {
                    printf("エラー: マッパーオプションが複数指定されています。\n");
                    return 1;
                }
                mapper = (opt == L"A8") ? &MAPPER_ASCII8 : (opt == L"A16") ? &MAPPER_ASCII16 : &MAPPER_KONAMI8;
            }
            else if (opt.rfind(L"P:", 0) == 0 && opt.size() > 2)
            {
                optPort = opt.substr(2);
            }
            else if (opt == L"P" && i + 1 < argc)
            {
                optPort = argv[++i];
            }
            else if (opt.rfind(L"S:", 0) == 0 && opt.size() > 2)
            {
                optSlot = _wtoi(opt.substr(2).c_str());
            }
            else if (opt == L"Y")
            {
                optYes = true;
            }
            else if (opt == L"NV")
            {
                optNoVerify = true;
            }
            else if (opt == L"?" || opt == L"H" || opt == L"HELP")
            {
                PrintUsage(argv[0]);
                return 0;
            }
            else
            {
                printf("エラー: 不明なオプション '%s'\n", ToUtf8(arg).c_str());
                PrintUsage(argv[0]);
                return 1;
            }
        }
        else
        {
            if (!romPath.empty())
            {
                printf("エラー: ROM ファイルが複数指定されています。\n");
                return 1;
            }
            romPath = arg;
        }
    }

    if (romPath.empty() || mapper == NULL)
    {
        if (romPath.empty()) printf("エラー: ROM ファイルが指定されていません。\n");
        if (mapper == NULL)  printf("エラー: マッパーオプション (/A8, /A16, /K8) が必要です。\n");
        printf("\n");
        PrintUsage(argv[0]);
        return 1;
    }
    if (optSlot != 1 && optSlot != 2)
    {
        printf("エラー: スロット番号は 1 または 2 を指定してください。\n");
        return 1;
    }

    // ---- ROM ファイル読み込みと整合チェック ---------------------------------
    HANDLE hFile = CreateFileW(romPath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL,
                               OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE)
    {
        printf("エラー: ROM ファイルを開けません: %s\n", ToUtf8(romPath).c_str());
        return 1;
    }
    DWORD fileSize = GetFileSize(hFile, NULL);
    if (fileSize == INVALID_FILE_SIZE || fileSize == 0)
    {
        printf("エラー: ROM ファイルのサイズが不正です。\n");
        CloseHandle(hFile);
        return 1;
    }
    std::vector<uint8_t> rom(fileSize);
    DWORD readSize = 0;
    bool readOk = ReadFile(hFile, rom.data(), fileSize, &readSize, NULL) && readSize == fileSize;
    CloseHandle(hFile);
    if (!readOk)
    {
        printf("エラー: ROM ファイルを読み込めませんでした。\n");
        return 1;
    }

    if ((uint32_t)fileSize > mapper->maxRomSize)
    {
        printf("エラー: ROM サイズ %u バイトが %s の最大容量 %u バイトを超えています。\n",
                fileSize, mapper->fullName, mapper->maxRomSize);
        if (mapper == &MAPPER_KONAMI8)
            printf("        (Simple Mega Flash の KONAMI8K モードは 128KB までです)\n");
        return 1;
    }

    // バンク境界に満たない分は 0xFF で埋める (消去状態 = 無書込と同じ)
    size_t paddedSize = ((rom.size() + mapper->bankSize - 1) / mapper->bankSize) * mapper->bankSize;
    if (paddedSize != rom.size())
    {
        printf("注意: ROM サイズ %zu バイトをバンク境界 %zu バイトへ 0xFF で拡張します。\n",
                rom.size(), paddedSize);
        rom.resize(paddedSize, 0xFF);
    }
    const size_t bankCount = rom.size() / mapper->bankSize;

    printf("ROM ファイル : %s\n", ToUtf8(romPath).c_str());
    printf("サイズ       : %u バイト (%zu バンク x %u バイト)\n", fileSize, bankCount, mapper->bankSize);
    printf("マッパー     : %s (%s)\n", mapper->fullName, mapper->optName);
    printf("\n");

    // ---- アダプタ接続 -------------------------------------------------------
    std::wstring usePort;
    if (!optPort.empty())
    {
        std::string hver;
        printf("%s のアダプタを確認中...\n", ToUtf8(optPort).c_str());
        if (!ProbeAdapter(optPort.c_str(), hver))
        {
            printf("エラー: %s で Game Cartridge Adapter が応答しません。\n", ToUtf8(optPort).c_str());
            printf("        USB 接続と、他のソフトがポートを開いていないか確認してください。\n");
            return 1;
        }
        usePort = optPort;
    }
    else
    {
        printf("アダプタを自動検出中...\n");
        std::vector<ComPortInfo> ports = EnumerateComPorts();
        // 優先順位の高いポートから順に調べる (列挙時点で USB デバイスのみに
        // 絞られているので、Bluetooth 等の仮想ポートは決して開かない):
        //   優先度0: USB VID/PID がアダプタと一致するポート
        //   優先度1: その他の USB ポート (フォールバック)
        // どのポートを調べているか、フレンドリ名付きで1行ずつ表示する
        // (フレンドリ名は表示専用で、識別には使わない)。
        for (int pass = 0; pass < 2 && usePort.empty(); pass++)
        {
            for (const auto& p : ports)
            {
                if (p.priority != pass) continue;
                std::string label = ToUtf8(p.name);
                if (!p.friendlyName.empty())
                    label += " (" + ToUtf8(p.friendlyName) + ")";
                if (p.priority < 1)
                    printf("%s を確認中 (アダプタ候補)...\n", label.c_str());
                else
                    printf("%s を確認中...\n", label.c_str());
                fflush(stdout);
                std::string hver;
                if (ProbeAdapter(p.name.c_str(), hver))
                {
                    usePort = p.name;
                    break;
                }
            }
        }
        if (usePort.empty())
        {
            printf("エラー: Game Cartridge Adapter が見つかりません。\n");
            printf("        USB 接続を確認するか、/P:COMx でポートを指定してください。\n");
            return 1;
        }
    }
    printf("アダプタ     : %s\n", ToUtf8(usePort).c_str());

    Serial ser;
    if (!ser.Open(usePort.c_str()))
    {
        printf("エラー: %s を開けません。\n", ToUtf8(usePort).c_str());
        return 1;
    }

    std::string resp;
    if (!ser.Command("HVER", resp, 5000))
    {
        printf("エラー: アダプタが HVER に応答しません。\n");
        return 1;
    }
    printf("%s", resp.c_str());

    // ---- スロット選択 / カートリッジ確認 / 電源 ON --------------------------
    char cmdbuf[64];
    sprintf(cmdbuf, "SSEL,%d", optSlot);
    if (!ser.Command(cmdbuf, resp, 5000))
    {
        printf("エラー: スロット選択 (SSEL) が失敗しました。\n");
        return 1;
    }

    bool inserted = false;
    for (int attempt = 0; attempt < 3 && !inserted; attempt++)
    {
        if (!ser.Command("SCHK", resp, 5000))
        {
            printf("エラー: SCHK が失敗しました。\n");
            return 1;
        }
        if (resp.find('1') != std::string::npos)
        {
            inserted = true;
            break;
        }
        printf("カートリッジが検出できません。アダプタに Simple Mega Flash を挿して Enter を押してください (中止は Ctrl+C)...\n");
        std::wstring dummy;
        ReadStdinLine(dummy);
    }
    if (!inserted)
    {
        printf("エラー: カートリッジが検出できませんでした。処理を中止します。\n");
        return 1;
    }

    if (!ser.Command("SPON", resp, 10000))
    {
        printf("エラー: スロット電源 ON (SPON) が失敗しました。過電流の可能性があります。\n");
        return 1;
    }
    ser.Command("SRST", resp, 5000);

    // 以降の失敗時は SPOFF してから終了する
    int exitCode = 1;
    bool powerOn = true;
    auto Shutdown = [&]() {
        if (powerOn)
        {
            std::string r;
            ser.Command("SPOFF", r, 5000);
            powerOn = false;
        }
    };

    // ---- Flash ID 確認 (消去前に必ず確認する) --------------------------------
    uint8_t maker = 0, device = 0;
    if (!FlashReadId(ser, maker, device))
    {
        printf("エラー: Flash ID を読み取れませんでした。\n");
        Shutdown();
        return 1;
    }
    printf("Flash ID     : Maker=%02X Device=%02X\n", maker, device);
    uint32_t flashCapacity = 0;
    const char* flashName = "";
    if (device == 0xB5) { flashCapacity = 128 * 1024; flashName = "SST39SF010A"; }
    if (device == 0xB6) { flashCapacity = 256 * 1024; flashName = "SST39SF020A"; }
    if (device == 0xB7) { flashCapacity = 512 * 1024; flashName = "SST39SF040A"; }
    if (maker != 0xBF || flashCapacity == 0)
    {
        printf("エラー: Simple Mega Flash (SST39SF 系) が見つかりません。\n");
        printf("        カートリッジの種類とスイッチ設定 (SW1=4Mbit 位置) を確認してください。\n");
        printf("        ※ 消去は行っていません。\n");
        Shutdown();
        return 1;
    }
    printf("Flash        : %s (%u バイト)\n", flashName, flashCapacity);
    if (rom.size() > flashCapacity)
    {
        printf("エラー: ROM サイズ %zu バイトが Flash 容量 %u バイトを超えています。\n",
                rom.size(), flashCapacity);
        Shutdown();
        return 1;
    }

    // ---- 消去前の確認 --------------------------------------------------------
    if (!optYes)
    {
        printf("\n上記の内容でカートリッジを消去して書き込みます。よろしいですか？ [y/N] ");
        fflush(stdout);
        std::wstring answer;
        ReadStdinLine(answer);
        if (answer != L"y" && answer != L"Y")
        {
            printf("中止しました (消去は行っていません)。\n");
            Shutdown();
            return 1;
        }
    }

    ULONGLONG startTick = GetTickCount64();

    // ---- チップ消去 ----------------------------------------------------------
    printf("\n消去中... ");
    fflush(stdout);
    if (!FlashChipErase(ser))
    {
        printf("\nエラー: チップ消去が完了しませんでした (タイムアウト)。\n");
        Shutdown();
        return 1;
    }
    printf("完了\n");

    // スクリプトのポーリング上限を引き上げておく
    if (!ser.Command("LSCR,FFFF", resp, 5000))
    {
        printf("エラー: LSCR の設定が失敗しました。\n");
        Shutdown();
        return 1;
    }

    // ---- 書き込み ------------------------------------------------------------
    printf("書き込み中... (全 %zu バンク)\n", bankCount);
    const size_t CHUNK = 512;
    std::vector<uint8_t> script;
    script.reserve(CHUNK * 36 + 4);
    size_t totalProgrammed = 0, totalSkipped = 0;

    for (size_t bank = 0; bank < bankCount; bank++)
    {
        const uint8_t* bankData = rom.data() + bank * mapper->bankSize;
        uint8_t segment = (uint8_t)bank;
        for (size_t off = 0; off < mapper->bankSize; off += CHUNK)
        {
            size_t programmed = BuildChunkScript(script, *mapper, segment,
                                                 bankData + off, CHUNK,
                                                 (uint16_t)(mapper->winBase + off));
            totalSkipped += (CHUNK - programmed);
            if (programmed == 0) continue;  // 全て 0xFF なら書く必要がない
            totalProgrammed += programmed;

            char brcv[32];
            sprintf(brcv, "BRCV,0,%X", (unsigned int)script.size());
            if (!ser.SendRaw(brcv) || !ser.WriteBinary(script.data(), script.size()))
            {
                printf("\nエラー: スクリプト送信に失敗しました (バンク %zu, オフセット 0x%04zX)。\n", bank, off);
                Shutdown();
                return 1;
            }
            if (!ser.WaitResult(resp, 30000))
            {
                printf("\nエラー: スクリプト受信 (BRCV) が失敗しました (バンク %zu, オフセット 0x%04zX)。\n", bank, off);
                Shutdown();
                return 1;
            }
            if (!ser.Command("BSCR,0", resp, 120000))
            {
                printf("\nエラー: 書き込みスクリプトが失敗しました (バンク %zu, オフセット 0x%04zX)。\n", bank, off);
                printf("        スイッチ設定 (SW2=%s, SW1=4Mbit) とカートリッジの接触を確認してください。\n", mapper->fullName);
                Shutdown();
                return 1;
            }
        }
        printf("\r  バンク %3zu / %zu 完了", bank + 1, bankCount);
        fflush(stdout);
    }
    printf("\n書き込み完了: %zu バイト書込, %zu バイトは 0xFF のため省略\n", totalProgrammed, totalSkipped);

    // ---- ベリファイ ----------------------------------------------------------
    if (!optNoVerify)
    {
        printf("ベリファイ中...\n");
        std::vector<uint8_t> readBack(mapper->bankSize);
        for (size_t bank = 0; bank < bankCount; bank++)
        {
            if (!ser.WriteByte(mapper->regAddr, (uint8_t)bank))
            {
                printf("エラー: ベリファイ時のバンク選択に失敗しました (バンク %zu)。\n", bank);
                Shutdown();
                return 1;
            }
            char smtr[48];
            sprintf(smtr, "SMTR,%04X,%04X,0,%d", mapper->winBase, mapper->bankSize, optSlot);
            if (!ser.Command(smtr, resp, 60000))
            {
                printf("エラー: 読み戻し転送 (SMTR) が失敗しました (バンク %zu)。\n", bank);
                Shutdown();
                return 1;
            }
            char bsnd[32];
            sprintf(bsnd, "BSND,0,%04X", mapper->bankSize);
            if (!ser.SendRaw(bsnd) ||
                !ser.ReadBinary(readBack.data(), readBack.size(), 60000) ||
                !ser.WaitResult(resp, 10000))
            {
                printf("エラー: 読み戻し受信 (BSND) が失敗しました (バンク %zu)。\n", bank);
                Shutdown();
                return 1;
            }
            const uint8_t* expect = rom.data() + bank * mapper->bankSize;
            for (size_t i = 0; i < mapper->bankSize; i++)
            {
                if (readBack[i] != expect[i])
                {
                    printf("エラー: ベリファイ不一致 バンク %zu, オフセット 0x%04zX: 期待 %02X / 実際 %02X\n",
                            bank, i, expect[i], readBack[i]);
                    Shutdown();
                    return 1;
                }
            }
            printf("\r  バンク %3zu / %zu OK", bank + 1, bankCount);
            fflush(stdout);
        }
        printf("\nベリファイ OK: 全 %zu バイト一致\n", rom.size());
    }

    Shutdown();
    exitCode = 0;
    printf("\n完了しました (%llu 秒)。カートリッジを取り外せます。\n",
            (unsigned long long)((GetTickCount64() - startTick) / 1000));
    return exitCode;
}
