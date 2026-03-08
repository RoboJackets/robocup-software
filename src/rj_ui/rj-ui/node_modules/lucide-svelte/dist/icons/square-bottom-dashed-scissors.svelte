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
const iconNode = [["line", { "x1": "5", "y1": "3", "x2": "19", "y2": "3" }], ["line", { "x1": "3", "y1": "5", "x2": "3", "y2": "19" }], ["line", { "x1": "21", "y1": "5", "x2": "21", "y2": "19" }], ["line", { "x1": "9", "y1": "21", "x2": "10", "y2": "21" }], ["line", { "x1": "14", "y1": "21", "x2": "15", "y2": "21" }], ["path", { "d": "M 3 5 A2 2 0 0 1 5 3" }], ["path", { "d": "M 19 3 A2 2 0 0 1 21 5" }], ["path", { "d": "M 5 21 A2 2 0 0 1 3 19" }], ["path", { "d": "M 21 19 A2 2 0 0 1 19 21" }], ["circle", { "cx": "8.5", "cy": "8.5", "r": "1.5" }], ["line", { "x1": "9.56066", "y1": "9.56066", "x2": "12", "y2": "12" }], ["line", { "x1": "17", "y1": "17", "x2": "14.82", "y2": "14.82" }], ["circle", { "cx": "8.5", "cy": "15.5", "r": "1.5" }], ["line", { "x1": "9.56066", "y1": "14.43934", "x2": "17", "y2": "7" }]];
/**
 * @component @name SquareBottomDashedScissors
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8bGluZSB4MT0iNSIgeTE9IjMiIHgyPSIxOSIgeTI9IjMiIC8+CiAgPGxpbmUgeDE9IjMiIHkxPSI1IiB4Mj0iMyIgeTI9IjE5IiAvPgogIDxsaW5lIHgxPSIyMSIgeTE9IjUiIHgyPSIyMSIgeTI9IjE5IiAvPgogIDxsaW5lIHgxPSI5IiB5MT0iMjEiIHgyPSIxMCIgeTI9IjIxIiAvPgogIDxsaW5lIHgxPSIxNCIgeTE9IjIxIiB4Mj0iMTUiIHkyPSIyMSIgLz4KICA8cGF0aCBkPSJNIDMgNSBBMiAyIDAgMCAxIDUgMyIgLz4KICA8cGF0aCBkPSJNIDE5IDMgQTIgMiAwIDAgMSAyMSA1IiAvPgogIDxwYXRoIGQ9Ik0gNSAyMSBBMiAyIDAgMCAxIDMgMTkiIC8+CiAgPHBhdGggZD0iTSAyMSAxOSBBMiAyIDAgMCAxIDE5IDIxIiAvPgogIDxjaXJjbGUgY3g9IjguNSIgY3k9IjguNSIgcj0iMS41IiAvPgogIDxsaW5lIHgxPSI5LjU2MDY2IiB5MT0iOS41NjA2NiIgeDI9IjEyIiB5Mj0iMTIiIC8+CiAgPGxpbmUgeDE9IjE3IiB5MT0iMTciIHgyPSIxNC44MiIgeTI9IjE0LjgyIiAvPgogIDxjaXJjbGUgY3g9IjguNSIgY3k9IjE1LjUiIHI9IjEuNSIgLz4KICA8bGluZSB4MT0iOS41NjA2NiIgeTE9IjE0LjQzOTM0IiB4Mj0iMTciIHkyPSI3IiAvPgo8L3N2Zz4K) - https://lucide.dev/icons/square-bottom-dashed-scissors
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
</script>

<Icon name="square-bottom-dashed-scissors" {...$$props} iconNode={iconNode}>
  <slot/>
</Icon>
