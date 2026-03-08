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
const iconNode = [["path", { "d": "m10 16 1.5 1.5" }], ["path", { "d": "m14 8-1.5-1.5" }], ["path", { "d": "M15 2c-1.798 1.998-2.518 3.995-2.807 5.993" }], ["path", { "d": "m16.5 10.5 1 1" }], ["path", { "d": "m17 6-2.891-2.891" }], ["path", { "d": "M2 15c6.667-6 13.333 0 20-6" }], ["path", { "d": "m20 9 .891.891" }], ["path", { "d": "M3.109 14.109 4 15" }], ["path", { "d": "m6.5 12.5 1 1" }], ["path", { "d": "m7 18 2.891 2.891" }], ["path", { "d": "M9 22c1.798-1.998 2.518-3.995 2.807-5.993" }]];
/**
 * @component @name Dna
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyAgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIgogIHdpZHRoPSIyNCIKICBoZWlnaHQ9IjI0IgogIHZpZXdCb3g9IjAgMCAyNCAyNCIKICBmaWxsPSJub25lIgogIHN0cm9rZT0iIzAwMCIgc3R5bGU9ImJhY2tncm91bmQtY29sb3I6ICNmZmY7IGJvcmRlci1yYWRpdXM6IDJweCIKICBzdHJva2Utd2lkdGg9IjIiCiAgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIgogIHN0cm9rZS1saW5lam9pbj0icm91bmQiCj4KICA8cGF0aCBkPSJtMTAgMTYgMS41IDEuNSIgLz4KICA8cGF0aCBkPSJtMTQgOC0xLjUtMS41IiAvPgogIDxwYXRoIGQ9Ik0xNSAyYy0xLjc5OCAxLjk5OC0yLjUxOCAzLjk5NS0yLjgwNyA1Ljk5MyIgLz4KICA8cGF0aCBkPSJtMTYuNSAxMC41IDEgMSIgLz4KICA8cGF0aCBkPSJtMTcgNi0yLjg5MS0yLjg5MSIgLz4KICA8cGF0aCBkPSJNMiAxNWM2LjY2Ny02IDEzLjMzMyAwIDIwLTYiIC8+CiAgPHBhdGggZD0ibTIwIDkgLjg5MS44OTEiIC8+CiAgPHBhdGggZD0iTTMuMTA5IDE0LjEwOSA0IDE1IiAvPgogIDxwYXRoIGQ9Im02LjUgMTIuNSAxIDEiIC8+CiAgPHBhdGggZD0ibTcgMTggMi44OTEgMi44OTEiIC8+CiAgPHBhdGggZD0iTTkgMjJjMS43OTgtMS45OTggMi41MTgtMy45OTUgMi44MDctNS45OTMiIC8+Cjwvc3ZnPgo=) - https://lucide.dev/icons/dna
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
</script>

<Icon name="dna" {...$$props} iconNode={iconNode}>
  <slot/>
</Icon>
