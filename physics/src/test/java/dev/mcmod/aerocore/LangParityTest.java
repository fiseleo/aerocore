package dev.mcmod.aerocore;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.jupiter.api.Assertions.*;

/**
 * AeroCore's three languages (0.41: English, Traditional and Simplified Chinese) have the same keys in the same order
 * and the same format arguments, and zh_cn (scripts/zh_cn.py makes it from zh_tw) has no Traditional characters left.
 * The game's resources are the neoforge project's (the test runs in the physics project).
 */
class LangParityTest {
    private static final Path LANG = Path.of("..", "neoforge", "src", "main", "resources", "assets", "aerocore", "lang");
    private static final Pattern ENTRY = Pattern.compile("^\\s*\"([^\"]+)\":\\s*\"((?:[^\"\\\\]|\\\\.)*)\",?\\s*$");
    private static final Pattern ARGUMENT = Pattern.compile("%(\\d+\\$)?[sd%]");
    /** Characters written only in Traditional Chinese (their Simplified forms differ), common in AeroCore's words. */
    private static final String TRADITIONAL = "這們個來說時對會後還發過車機電體設計選擇項鍵開關門錯誤當號碼點線級組裝與態從將數據顯環動變換"
            + "實際長寬間題節燈閥軸輪轉頭壓氣燒噴製廠產業處圖藍標準鋼鐵銅鋁鎳鋅錫鉻鈦鎂礦煉爐齒紋"
            + "漿區場臺風範網絡頁籤畫層邊緣尋讀寫儲檔視螢資訊條優勢創務協議運輸載飛艙駕駛輛帶濾測驗淺";

    private static Map<String, String> read(String language) throws IOException {
        Map<String, String> out = new LinkedHashMap<>();
        for (String line : Files.readAllLines(LANG.resolve(language + ".json"), StandardCharsets.UTF_8)) {
            Matcher m = ENTRY.matcher(line);
            if (m.matches()) { assertNull(out.put(m.group(1), m.group(2)), language + " has " + m.group(1) + " once"); }
        }
        return out;
    }

    private static List<String> arguments(String value) {
        List<String> out = new ArrayList<>();
        Matcher m = ARGUMENT.matcher(value);
        while (m.find()) { out.add(m.group()); }
        out.sort(null);
        return out;
    }

    @Test void theThreeLanguagesHaveTheSameKeysAndArguments() throws IOException {
        Map<String, String> en = read("en_us"), tw = read("zh_tw"), cn = read("zh_cn");
        assertTrue(en.size() > 100, "the language files are read");
        assertEquals(List.copyOf(en.keySet()), List.copyOf(tw.keySet()), "zh_tw: the same keys in the same order");
        assertEquals(List.copyOf(en.keySet()), List.copyOf(cn.keySet()), "zh_cn: the same keys in the same order");
        for (String key : en.keySet()) {
            assertEquals(arguments(en.get(key)), arguments(tw.get(key)), "zh_tw " + key + " takes the same arguments");
            assertEquals(arguments(tw.get(key)), arguments(cn.get(key)), "zh_cn " + key + " takes the same arguments");
        }
    }

    @Test void simplifiedChineseHasNoTraditionalCharacters() throws IOException {
        List<String> left = new ArrayList<>();
        for (var e : read("zh_cn").entrySet()) {
            for (char c : e.getValue().toCharArray()) {
                if (TRADITIONAL.indexOf(c) >= 0) { left.add(e.getKey() + " '" + c + "'"); break; }
            }
        }
        assertEquals(List.of(), left, "run scripts/zh_cn.py after changing zh_tw");
    }
}
