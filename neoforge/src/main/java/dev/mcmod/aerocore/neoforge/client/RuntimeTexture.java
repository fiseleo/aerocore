package dev.mcmod.aerocore.neoforge.client;

import com.mojang.blaze3d.platform.GlStateManager;
import com.mojang.blaze3d.platform.NativeImage;
import com.mojang.blaze3d.platform.TextureUtil;
import com.mojang.blaze3d.systems.RenderSystem;
import net.minecraft.client.renderer.texture.AbstractTexture;
import net.minecraft.client.renderer.texture.MipmapGenerator;
import net.minecraft.server.packs.resources.ResourceManager;
import org.lwjgl.opengl.GL11;

import java.util.Objects;

/**
 * A texture made in the game (0.31), for {@link PartFinishes#define}: a picture a player uploaded, say. Uploaded like a
 * finish's PNG (the whole mip chain, anisotropic where the driver has it), clamped at its edges when asked (a decal
 * does not repeat; its far edge would bleed into the near one). It keeps its picture, so the resource reload that loads
 * every registered texture again uploads it again; {@link #replace} swaps the picture in place, so the render types
 * that bind it stay valid and nothing is baked again. Render thread.
 */
public final class RuntimeTexture extends AbstractTexture {
    private static final int GL_TEXTURE_MAX_ANISOTROPY = 0x84FE;
    private NativeImage image;
    private final boolean clamp;

    /** Takes the image: it is closed with the texture or when replaced. */
    public RuntimeTexture(NativeImage image, boolean clamp) {
        this.image = Objects.requireNonNull(image, "image");
        this.clamp = clamp;
    }

    @Override public void load(ResourceManager manager) { upload(); }

    /** Swaps the picture (the old one is closed) and uploads it; a different size is fine. */
    public void replace(NativeImage next) {
        RenderSystem.assertOnRenderThreadOrInit();
        NativeImage old = image;
        image = Objects.requireNonNull(next, "image");
        if (old != next) { old.close(); }
        upload();
    }

    public int width() { return image.getWidth(); }
    public int height() { return image.getHeight(); }

    private void upload() {
        RenderSystem.assertOnRenderThreadOrInit();
        int w = image.getWidth(), h = image.getHeight();
        int levels = 0;
        while ((w >> (levels + 1)) >= 1 && (h >> (levels + 1)) >= 1 && levels < 8) { levels++; }
        NativeImage[] mips = MipmapGenerator.generateMipLevels(new NativeImage[]{image}, levels);
        TextureUtil.prepareImage(getId(), levels, w, h);
        for (int i = 0; i <= levels; i++) {
            // the picture itself is kept (a reload uploads it again); the smaller levels are made for this upload
            mips[i].upload(i, 0, 0, 0, 0, Math.max(1, w >> i), Math.max(1, h >> i), true, clamp, true, i > 0);
        }
        float a = FinishTexture.maxAnisotropy();
        if (a > 1) {
            bind();
            GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, a);
        }
        blur = true;
        mipmap = true;
    }

    @Override public void close() { image.close(); }
}
