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
const iconNode = [["path", { "d": "M11.146 15.854a1.207 1.207 0 0 1 1.708 0l1.56 1.56A2 2 0 0 1 15 18.828V21a1 1 0 0 1-1 1h-4a1 1 0 0 1-1-1v-2.172a2 2 0 0 1 .586-1.414z" }], ["path", { "d": "M18.828 15a2 2 0 0 1-1.414-.586l-1.56-1.56a1.207 1.207 0 0 1 0-1.708l1.56-1.56A2 2 0 0 1 18.828 9H21a1 1 0 0 1 1 1v4a1 1 0 0 1-1 1z" }], ["path", { "d": "M6.586 14.414A2 2 0 0 1 5.172 15H3a1 1 0 0 1-1-1v-4a1 1 0 0 1 1-1h2.172a2 2 0 0 1 1.414.586l1.56 1.56a1.207 1.207 0 0 1 0 1.708z" }], ["path", { "d": "M9 3a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v2.172a2 2 0 0 1-.586 1.414l-1.56 1.56a1.207 1.207 0 0 1-1.708 0l-1.56-1.56A2 2 0 0 1 9 5.172z" }]];
/**
 * @component @name GamepadDirectional
 * @description Lucide SVG icon component, renders SVG Element with children.
 *
 * @preview ![img](data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyNCIgaGVpZ2h0PSIyNCIgdmlld0JveD0iMCAwIDI0IDI0IiBmaWxsPSJub25lIiBzdHJva2U9IiMwMDAiIHN0eWxlPSJiYWNrZ3JvdW5kLWNvbG9yOiAjZmZmOyBib3JkZXItcmFkaXVzOiAycHgiICBzdHJva2Utd2lkdGg9IjIiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgc3Ryb2tlLWxpbmVqb2luPSJyb3VuZCI+CiAgPHBhdGgKICAgIGQ9Ik0xMS4xNDYgMTUuODU0YTEuMjA3IDEuMjA3IDAgMCAxIDEuNzA4IDBsMS41NiAxLjU2QTIgMiAwIDAgMSAxNSAxOC44MjhWMjFhMSAxIDAgMCAxLTEgMWgtNGExIDEgMCAwIDEtMS0xdi0yLjE3MmEyIDIgMCAwIDEgLjU4Ni0xLjQxNHoiIC8+CiAgPHBhdGgKICAgIGQ9Ik0xOC44MjggMTVhMiAyIDAgMCAxLTEuNDE0LS41ODZsLTEuNTYtMS41NmExLjIwNyAxLjIwNyAwIDAgMSAwLTEuNzA4bDEuNTYtMS41NkEyIDIgMCAwIDEgMTguODI4IDlIMjFhMSAxIDAgMCAxIDEgMXY0YTEgMSAwIDAgMS0xIDF6IiAvPgogIDxwYXRoCiAgICBkPSJNNi41ODYgMTQuNDE0QTIgMiAwIDAgMSA1LjE3MiAxNUgzYTEgMSAwIDAgMS0xLTF2LTRhMSAxIDAgMCAxIDEtMWgyLjE3MmEyIDIgMCAwIDEgMS40MTQuNTg2bDEuNTYgMS41NmExLjIwNyAxLjIwNyAwIDAgMSAwIDEuNzA4eiIgLz4KICA8cGF0aAogICAgZD0iTTkgM2ExIDEgMCAwIDEgMS0xaDRhMSAxIDAgMCAxIDEgMXYyLjE3MmEyIDIgMCAwIDEtLjU4NiAxLjQxNGwtMS41NiAxLjU2YTEuMjA3IDEuMjA3IDAgMCAxLTEuNzA4IDBsLTEuNTYtMS41NkEyIDIgMCAwIDEgOSA1LjE3MnoiIC8+Cjwvc3ZnPg==) - https://lucide.dev/icons/gamepad-directional
 * @see https://lucide.dev/guide/packages/lucide-svelte - Documentation
 *
 * @param {Object} props - Lucide icons props and any valid SVG attribute
 * @returns {FunctionalComponent} Svelte component
 *
 */
</script>

<Icon name="gamepad-directional" {...$$props} iconNode={iconNode}>
  <slot/>
</Icon>
