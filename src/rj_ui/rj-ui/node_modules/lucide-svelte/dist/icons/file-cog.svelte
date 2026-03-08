<script>/**
 * @license lucide-svelte v0.562.0 - ISC
 *
 * ISC License
 *
 * Copyright (c) for portions of Lucide are held by Cole Bemis 2013-2023 as part of Feather (MIT). All other copyright (c) for Lucide are held by Lucide Contributors 2025.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * ---
 *
 * The MIT License (MIT) (for portions derived from Feather)
 *
 * Copyright (c) 2013-2023 Cole Bemis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
import Icon from '../Icon.svelte';
const iconNode = [["path", { "d": "M13.85 22H18a2 2 0 0 0 2-2V8a2 2 0 0 0-.586-1.414l-4-4A2 2 0 0 0 14 2H6a2 2 0 0 0-2 2v6.6" }], ["path", { "d": "M14 2v5a1 1 0 0 0 1 1h5" }], ["path", { "d": "m3.305 19.53.923-.382" }], ["path", { "d": "m4.228 16.852-.924-.383" }], ["path", { "d": "m5.852 15.228-.383-.923" }], ["path", { "d": "m5.852 20.772-.383.924" }], ["path", { "d": "m8.148 15.228.383-.923" }], ["path", { "d": "m8.53 21.696-.382-.924" }], ["path", { "d": "m9.773 16.852.922-.383" }], ["path", { "d": "m9.773 19.148.922.383" }], ["circle", { "cx": "7", "cy": "18", "r": "3" }]];
/**
 * @component @name FileCog
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8cGF0aCBkPSJNMTMuODUgMjJIMThhMiAyIDAgMCAwIDItMlY4YTIgMiAwIDAgMC0uNTg2LTEuNDE0bC00LTRBMiAyIDAgMCAwIDE0IDJINmEyIDIgMCAwIDAtMiAydjYuNiIgLz4KICA8cGF0aCBkPSJNMTQgMnY1YTEgMSAwIDAgMCAxIDFoNSIgLz4KICA8cGF0aCBkPSJtMy4zMDUgMTkuNTMuOTIzLS4zODIiIC8+CiAgPHBhdGggZD0ibTQuMjI4IDE2Ljg1Mi0uOTI0LS4zODMiIC8+CiAgPHBhdGggZD0ibTUuODUyIDE1LjIyOC0uMzgzLS45MjMiIC8+CiAgPHBhdGggZD0ibTUuODUyIDIwLjc3Mi0uMzgzLjkyNCIgLz4KICA8cGF0aCBkPSJtOC4xNDggMTUuMjI4LjM4My0uOTIzIiAvPgogIDxwYXRoIGQ9Im04LjUzIDIxLjY5Ni0uMzgyLS45MjQiIC8+CiAgPHBhdGggZD0ibTkuNzczIDE2Ljg1Mi45MjItLjM4MyIgLz4KICA8cGF0aCBkPSJtOS43NzMgMTkuMTQ4LjkyMi4zODMiIC8+CiAgPGNpcmNsZSBjeD0iNyIgY3k9IjE4IiByPSIzIiAvPgo8L3N2Zz4K) - https://lucide.dev/icons/file-cog
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
</script>

<Icon name="file-cog" {...$$props} iconNode={iconNode}>
  <slot/>
</Icon>
