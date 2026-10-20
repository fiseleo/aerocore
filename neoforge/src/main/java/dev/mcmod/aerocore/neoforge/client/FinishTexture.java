package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.GlStateManager;
import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.platform.TextureUtil;
import net.minecraft.client.renderer.texture.AbstractTexture;
import net.minecraft.client.renderer.texture.MipmapGenerator;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.packs.resources.Resource;
import net.minecraft.server.packs.resources.ResourceManager;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;

import java.io.IOException;
import java.io.InputStream;

/**
 * A finish's tiling texture (0.29): read from a PNG, uploaded with its whole mip chain, wrapping (repeat), and with
 * anisotropic filtering where the driver has it. Minecraft's own entity textures upload one level and sample the
 * nearest texel, which shimmers on a tiled metal surface seen at a slant; render types for finishes bind this with
 * blur and mipmaps on (trilinear).
 */
public final class FinishTexture extends AbstractTexture {
    private static final int GL_TEXTURE_MAX_ANISOTROPY = 0x84FE, GL_MAX_TEXTURE_MAX_ANISOTROPY = 0x84FF;
    private static Float anisotropy;
    private final ResourceLocation source;

    public FinishTexture(ResourceLocation source) { this.source = source; }

    @Override public void load(ResourceManager manager) throws IOException {
        Resource resource = manager.getResourceOrThrow(source);
        NativeImage image;
        try (InputStream in = resource.open()) { image = NativeImage.read(in); }
        int w = image.getWidth(), h = image.getHeight();
        int levels = 0;
        while ((w >> (levels + 1)) >= 1 && (h >> (levels + 1)) >= 1 && levels < 8) { levels++; }
        NativeImage[] mips = MipmapGenerator.generateMipLevels(new NativeImage[]{image}, levels);
        TextureUtil.prepareImage(getId(), levels, w, h);
        for (int i = 0; i <= levels; i++) {
            mips[i].upload(i, 0, 0, 0, 0, Math.max(1, w >> i), Math.max(1, h >> i), true, false, true, true);
        }
        float a = maxAnisotropy();
        if (a > 1) {
            bind();
            GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, a);
        }
        blur = true;
        mipmap = true;
    }

    /** Up to 8, where the extension (or GL 4.6) is there; 1 without. */
    static float maxAnisotropy() {
        if (anisotropy == null) {
            var caps = GL.getCapabilities();
            anisotropy = caps.GL_EXT_texture_filter_anisotropic || caps.GL_ARB_texture_filter_anisotropic
                    ? Math.min(8f, GL11.glGetFloat(GL_MAX_TEXTURE_MAX_ANISOTROPY)) : 1f;
        }
        return anisotropy;
    }
}
