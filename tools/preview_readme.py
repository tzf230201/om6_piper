"""Render the local README for visual QA. No server, network or hardware access."""
from pathlib import Path
import shutil
from markdown_it import MarkdownIt

REPO = Path(__file__).resolve().parents[1]


def main():
    output = REPO / '.local/readme_preview'
    output.mkdir(parents=True, exist_ok=True)
    images = output / 'docs/images'
    images.mkdir(parents=True, exist_ok=True)
    for image in (REPO / 'docs/images').iterdir():
        if image.suffix.lower() in ('.svg', '.png', '.jpg'):
            shutil.copy2(image, images / image.name)
    renderer = MarkdownIt('commonmark', {'html': True}).enable('table')
    body = renderer.render((REPO / 'README.md').read_text(encoding='utf-8'))
    html = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OM6 PiPER — README preview</title><style>
*{box-sizing:border-box}body{margin:0;background:#f6f8fa;color:#1f2328;font:16px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}
header{border-bottom:1px solid #d0d7de;background:#fff;padding:14px 32px;color:#59636e;font-size:14px}
main{background:white;max-width:980px;margin:28px auto;padding:34px 44px;border:1px solid #d0d7de;border-radius:8px}
h1{font-size:34px;margin:0 0 16px;border-bottom:1px solid #d8dee4;padding-bottom:12px}h2{font-size:25px;border-bottom:1px solid #d8dee4;margin:32px 0 16px;padding-bottom:8px}h3{font-size:20px;margin-top:24px}
p{margin:14px 0}a{color:#0969da;text-decoration:none}img{max-width:100%;height:auto;border-radius:4px}table{border-collapse:collapse;width:100%;font-size:14px;margin:16px 0}td,th{border:1px solid #d0d7de;padding:10px 14px;text-align:left;vertical-align:top}tr:nth-child(even){background:#f6f8fa}
pre{overflow:auto;border-radius:6px;padding:18px;background:#f6f8fa;font-size:13px;line-height:1.65}code{font-family:Consolas,monospace;background:#eff1f3;border-radius:4px;padding:.15em .3em;font-size:.87em}pre code{background:none;padding:0;font-size:inherit}
blockquote{border-left:4px solid #b8c8d1;color:#59636e;padding:0 18px;margin:20px 0}li{margin:7px 0}summary{cursor:pointer;color:#0969da}em{color:#59636e;font-size:14px}
@media(max-width:700px){main{border:0;margin:0;padding:20px}h1{font-size:28px}}
</style><header>om6_piper / README.md · Local documentation preview</header><main>'''
    target = output / 'index.html'
    target.write_text(html + body + '</main></html>', encoding='utf-8')
    print(target)


if __name__ == '__main__':
    main()
