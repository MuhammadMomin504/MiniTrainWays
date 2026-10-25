using System.IO;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Generates the road textures and the shared road material used by SplineTrack:
/// a seamlessly tiling asphalt texture (mapped from the ground plane by the shader) and a
/// lane-marking texture laid out across one road width (U) and one marking repeat (V).
/// </summary>
public static class RoadSurfaceGenerator
{
    public const string Folder = "Assets/Road";
    public const string AsphaltPath = Folder + "/RoadAsphalt.png";
    public const string MarkingsPath = Folder + "/RoadMarkings.png";
    public const string MaterialPath = Folder + "/RoadSurface.mat";
    const string ShaderName = "MiniTrainWays/Road Surface";

    const int AsphaltSize = 512;
    const int MarkingsWidth = 256;
    const int MarkingsLength = 512;

    // Marking layout across the road (0 = left edge, 1 = right edge) and along one repeat.
    const float EdgeLineInner = 0.06f;
    const float EdgeLineOuter = 0.11f;
    const float CenterLineHalfWidth = 0.03f;
    const float DashStart = 0.25f;
    const float DashEnd = 0.75f;

    static readonly Color EdgePaint = new Color(0.93f, 0.93f, 0.9f);
    static readonly Color CenterPaint = new Color(0.96f, 0.78f, 0.22f);

    [MenuItem("Tools/MiniTrainWays/Generate Road Surface")]
    public static Material Generate()
    {
        Directory.CreateDirectory(Folder);

        File.WriteAllBytes(AsphaltPath, BuildAsphalt().EncodeToPNG());
        File.WriteAllBytes(MarkingsPath, BuildMarkings().EncodeToPNG());
        AssetDatabase.ImportAsset(AsphaltPath);
        AssetDatabase.ImportAsset(MarkingsPath);
        ConfigureImporter(AsphaltPath, TextureWrapMode.Repeat, TextureWrapMode.Repeat, false);
        ConfigureImporter(MarkingsPath, TextureWrapMode.Clamp, TextureWrapMode.Repeat, true);

        var material = AssetDatabase.LoadAssetAtPath<Material>(MaterialPath);
        var shader = Shader.Find(ShaderName);
        if (material == null)
        {
            material = new Material(shader) { name = "RoadSurface" };
            AssetDatabase.CreateAsset(material, MaterialPath);
        }
        else
        {
            material.shader = shader;
        }

        material.SetTexture("_AsphaltTex", AssetDatabase.LoadAssetAtPath<Texture2D>(AsphaltPath));
        material.SetTexture("_MarkingsTex", AssetDatabase.LoadAssetAtPath<Texture2D>(MarkingsPath));
        EditorUtility.SetDirty(material);
        AssetDatabase.SaveAssets();
        return material;
    }

    static void ConfigureImporter(string path, TextureWrapMode wrapU, TextureWrapMode wrapV, bool hasAlpha)
    {
        var importer = (TextureImporter)AssetImporter.GetAtPath(path);
        importer.textureType = TextureImporterType.Default;
        importer.sRGBTexture = true;
        importer.mipmapEnabled = true;
        importer.wrapModeU = wrapU;
        importer.wrapModeV = wrapV;
        importer.filterMode = FilterMode.Trilinear;
        importer.anisoLevel = 8;
        importer.alphaSource = hasAlpha ? TextureImporterAlphaSource.FromInput : TextureImporterAlphaSource.None;
        importer.alphaIsTransparency = hasAlpha;
        importer.SaveAndReimport();
    }

    static Texture2D BuildAsphalt()
    {
        var pixels = new Color[AsphaltSize * AsphaltSize];
        var random = new System.Random(1867);

        for (int y = 0; y < AsphaltSize; y++)
        {
            for (int x = 0; x < AsphaltSize; x++)
            {
                float u = x / (float)AsphaltSize;
                float v = y / (float)AsphaltSize;

                // Every octave's lattice period divides the texture, so the result tiles seamlessly.
                float body = 0f;
                float amplitude = 0.5f;
                float total = 0f;
                for (int period = 8; period <= 128; period *= 2)
                {
                    body += PeriodicNoise(u * period, v * period, period, period * 31) * amplitude;
                    total += amplitude;
                    amplitude *= 0.6f;
                }
                body /= total;

                float stains = PeriodicNoise(u * 4f, v * 4f, 4, 977);
                float grain = (float)random.NextDouble() - 0.5f;
                float value = 0.155f + (body - 0.5f) * 0.07f - (stains - 0.5f) * 0.03f + grain * 0.035f;
                pixels[y * AsphaltSize + x] = new Color(value, value, value * 1.04f, 1f);
            }
        }

        // Exposed aggregate: small light and dark chips, wrapped so they tile.
        for (int i = 0; i < 2600; i++)
        {
            int cx = random.Next(AsphaltSize);
            int cy = random.Next(AsphaltSize);
            float radius = 0.6f + (float)random.NextDouble() * 1.1f;
            bool light = random.NextDouble() < 0.7;
            float tone = light ? 0.26f + (float)random.NextDouble() * 0.12f : 0.07f + (float)random.NextDouble() * 0.04f;
            int r = Mathf.CeilToInt(radius);
            for (int dy = -r; dy <= r; dy++)
            {
                for (int dx = -r; dx <= r; dx++)
                {
                    float coverage = Mathf.Clamp01(radius + 0.5f - Mathf.Sqrt(dx * dx + dy * dy));
                    if (coverage <= 0f) continue;
                    int px = (cx + dx + AsphaltSize) % AsphaltSize;
                    int py = (cy + dy + AsphaltSize) % AsphaltSize;
                    int index = py * AsphaltSize + px;
                    var chip = new Color(tone, tone, tone * 1.03f, 1f);
                    pixels[index] = Color.Lerp(pixels[index], chip, coverage * 0.8f);
                }
            }
        }

        var texture = new Texture2D(AsphaltSize, AsphaltSize, TextureFormat.RGBA32, false);
        texture.SetPixels(pixels);
        texture.Apply();
        return texture;
    }

    static Texture2D BuildMarkings()
    {
        var pixels = new Color[MarkingsWidth * MarkingsLength];
        float pixelU = 1f / MarkingsWidth;
        float pixelV = 1f / MarkingsLength;

        for (int y = 0; y < MarkingsLength; y++)
        {
            for (int x = 0; x < MarkingsWidth; x++)
            {
                float u = (x + 0.5f) * pixelU;
                float v = (y + 0.5f) * pixelV;

                float edge = Mathf.Max(
                    Band(u, EdgeLineInner, EdgeLineOuter, pixelU),
                    Band(u, 1f - EdgeLineOuter, 1f - EdgeLineInner, pixelU));
                float center = Band(u, 0.5f - CenterLineHalfWidth, 0.5f + CenterLineHalfWidth, pixelU)
                               * Band(v, DashStart, DashEnd, pixelV * 2f);

                // Slightly worn paint, tiling along V.
                float wear = 0.8f + 0.2f * PeriodicNoise(u * 32f, v * 64f, 64, 4242);
                float alpha = Mathf.Max(edge, center) * wear;

                // Colour is defined everywhere so filtering never pulls dark fringes into the paint.
                Color paint = Mathf.Abs(u - 0.5f) < 0.25f ? CenterPaint : EdgePaint;
                paint.a = alpha;
                pixels[y * MarkingsWidth + x] = paint;
            }
        }

        var texture = new Texture2D(MarkingsWidth, MarkingsLength, TextureFormat.RGBA32, false);
        texture.SetPixels(pixels);
        texture.Apply();
        return texture;
    }

    // Anti-aliased 0..1 coverage of the interval [from, to] at coordinate p.
    static float Band(float p, float from, float to, float softness)
    {
        return Mathf.Clamp01((p - from) / softness + 0.5f) * Mathf.Clamp01((to - p) / softness + 0.5f);
    }

    static float PeriodicNoise(float x, float y, int period, int seed)
    {
        int x0 = Mathf.FloorToInt(x);
        int y0 = Mathf.FloorToInt(y);
        float fx = x - x0;
        float fy = y - y0;
        fx = fx * fx * (3f - 2f * fx);
        fy = fy * fy * (3f - 2f * fy);

        float a = Hash(x0, y0, period, seed);
        float b = Hash(x0 + 1, y0, period, seed);
        float c = Hash(x0, y0 + 1, period, seed);
        float d = Hash(x0 + 1, y0 + 1, period, seed);
        return Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, d, fx), fy);
    }

    static float Hash(int x, int y, int period, int seed)
    {
        x = ((x % period) + period) % period;
        y = ((y % period) + period) % period;
        unchecked
        {
            uint h = (uint)(x * 374761393 + y * 668265263 + seed * 2147483647);
            h = (h ^ (h >> 13)) * 1274126177u;
            h ^= h >> 16;
            return (h & 0xFFFFFF) / (float)0xFFFFFF;
        }
    }
}
