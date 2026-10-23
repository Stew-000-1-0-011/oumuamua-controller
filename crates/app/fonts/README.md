# Bundled font

`NotoSansJP-subset.otf` is a subset of the "Noto Sans CJK JP" Regular face
(Noto Sans CJK, Copyright 2014-2021 Adobe, SIL Open Font License 1.1, see `OFL.txt`),
taken from `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc` (Debian `fonts-noto-cjk`).

Subset: U+0020-007E, U+00A0-00FF, U+2000-206F, U+2190-21FF, U+2460-24FF, U+25A0-25FF,
U+3000-30FF, U+31F0-31FF, U+FF00-FFEF, plus every kanji in JIS X 0208 levels 1 and 2.

Regenerate: `python3 make_subset.py [path/to/NotoSansCJK-Regular.ttc]` (needs `fonttools`).

The app embeds it and points Slint's `SLINT_DEFAULT_FONT` at it (see `src/font.rs`), so kana
render on Android devices whose fallback font has kanji but no kana.

## Monospace font (console)

`ui/fonts/DejaVuSansMono-subset.ttf` / `ui/fonts/DejaVuSansMono-subset-Bold.ttf` are subsets of DejaVu Sans Mono
(Debian `fonts-dejavu-core`, license in `ui/fonts/DejaVu-LICENSE.txt`, about 28 KB each): U+0020-007E,
U+00A0-00FF, box drawing and block elements (U+2500-259F), arrows U+2190-2193, U+2022, U+2026.
`ui/console.slint` imports them as `fonts/...`, so Slint registers them (embedded with
`EmbedResourcesKind::EmbedFiles`, see `build.rs`) and `font-family: "DejaVu Sans Mono"` resolves
on Android too.

Regenerate: `pyftsubset DejaVuSansMono.ttf --unicodes=... --layout-features= --no-hinting
--desubroutinize --name-IDs='*' --output-file=DejaVuSansMono-subset.ttf` (needs `fonttools`).
