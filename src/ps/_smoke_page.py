import re
import live_view_raw as L

page = L.PAGE_TEMPLATE.replace("<!--MODES-->", L.build_mode_buttons())
ids = set(re.findall(r'id="([^"]+)"', page))
refs = set(re.findall(r"getElementById\('([^']+)'\)", page))
print("page %d bytes | ids %d | refs %d | missing %s" % (len(page), len(ids), len(refs), sorted(refs - ids) or "none"))
assert not (refs - ids)

src = open("live_view_raw.py", encoding="utf-8").read()
for token in ('id="sw"', "s.swipe", "--swipe-travel", "--swipe-cooldown", "args.auto", "SWIPE_DIAG"):
    print("%-18s %s" % (token, "present" if token in src or token in page else "MISSING"))

import session
s = session.GestureSession()
print("GestureSession OK; page mode buttons:", page.count('href="/mode/'))
