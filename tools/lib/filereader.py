from common.url_file import URLFile

def FileReader(fn, debug=False):
  if fn.startswith("http://") or fn.startswith("https://"):
    return URLFile(fn, debug=debug)
  else:
    return open(fn, "rb")
