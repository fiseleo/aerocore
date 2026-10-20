import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Map;
import java.util.TreeMap;

/**
 * Compares screenshots (0.39): two images, or two folders of AutoShot shots paired by name after their tag
 * ({@code aerocore-b22-sedan.png} with {@code aerocore-o22-sedan.png}). For each pair it prints the largest channel
 * difference, the share of pixels differing by more than 2 and by more than 16 levels, and writes a heat map (black
 * where equal, red by the difference) into the output folder. Run with the JDK, no build needed:
 * {@code java scripts/ShotDiff.java <a.png|folder> <b.png|folder> <out folder>}.
 */
public final class ShotDiff {
    public static void main(String[] args) throws Exception {
        if (args.length != 3) {
            System.err.println("java ShotDiff.java <a.png|folder> <b.png|folder> <out folder>");
            System.exit(2);
        }
        File a = new File(args[0]), b = new File(args[1]), out = new File(args[2]);
        out.mkdirs();
        Map<String, File[]> pairs = new TreeMap<>();
        if (a.isDirectory() && b.isDirectory()) {
            Map<String, File> as = byView(a), bs = byView(b);
            for (Map.Entry<String, File> e : as.entrySet()) {
                File other = bs.get(e.getKey());
                if (other != null) { pairs.put(e.getKey(), new File[]{e.getValue(), other}); }
                else { System.out.println("only in " + a + ": " + e.getValue().getName()); }
            }
            for (String k : bs.keySet()) { if (!as.containsKey(k)) { System.out.println("only in " + b + ": " + bs.get(k).getName()); } }
        } else {
            pairs.put(a.getName().replaceAll("\\.png$", ""), new File[]{a, b});
        }
        System.out.printf("%-28s %6s %9s %9s%n", "view", "max", ">2", ">16");
        for (Map.Entry<String, File[]> e : pairs.entrySet()) {
            BufferedImage x = ImageIO.read(e.getValue()[0]), y = ImageIO.read(e.getValue()[1]);
            if (x.getWidth() != y.getWidth() || x.getHeight() != y.getHeight()) {
                System.out.printf("%-28s sizes differ%n", e.getKey());
                continue;
            }
            int w = x.getWidth(), h = x.getHeight(), max = 0;
            long over2 = 0, over16 = 0;
            BufferedImage heat = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
            for (int j = 0; j < h; j++) {
                for (int i = 0; i < w; i++) {
                    int p = x.getRGB(i, j), q = y.getRGB(i, j), d = 0;
                    for (int s = 0; s <= 16; s += 8) { d = Math.max(d, Math.abs((p >> s & 255) - (q >> s & 255))); }
                    max = Math.max(max, d);
                    if (d > 2) { over2++; }
                    if (d > 16) { over16++; }
                    int r = Math.min(255, d * 4);
                    heat.setRGB(i, j, r << 16 | (d > 2 ? 40 : 0) << 8);
                }
            }
            double n = (double) w * h;
            System.out.printf("%-28s %6d %8.3f%% %8.3f%%%n", e.getKey(), max, 100 * over2 / n, 100 * over16 / n);
            ImageIO.write(heat, "png", new File(out, e.getKey() + "-diff.png"));
        }
    }

    /** A folder's PNGs by their view: the name less "aerocore-" and its tag ("aerocore-b22-sedan.png" is "sedan"). */
    private static Map<String, File> byView(File dir) {
        Map<String, File> m = new TreeMap<>();
        File[] files = dir.listFiles((d, n) -> n.endsWith(".png"));
        if (files == null) { return m; }
        for (File f : files) {
            String n = f.getName().replaceAll("\\.png$", "").replaceFirst("^aerocore-", "");
            int dash = n.indexOf('-');
            m.put(dash > 0 ? n.substring(dash + 1) : n, f);
        }
        return m;
    }
}
