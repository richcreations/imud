--[[
  imud — IMU daemon
  Copyright (c) 2026 Richard Simpson
  SPDX-License-Identifier: MIT

  texi-headings.lua — make docs/manual.md's headings usable as Texinfo nodes,
  and its links usable in Info.

  Three jobs.

  1. FLATTEN HEADINGS.  pandoc derives a @node name from the heading's
     INLINES, markup included, so `### `[device]`` becomes

         @node @code{[device]}

     and the Info menu a reader navigates by reads `* @code{[device]}::`.
     Info is plain text — @code buys nothing in a section title and costs a
     legible node name — so headings are flattened to their text.

     Non-ASCII is deliberately KEPT.  Texinfo 7 with @documentencoding UTF-8
     takes it without complaint on all 71 headings, and "specific force,
     m/s2" is a worse title than the one the manual wrote.  Collisions are
     the real risk of flattening, so this counts them and fails rather than
     letting two sections quietly share a node.

  2. RESOLVE INTRA-DOCUMENT LINKS OURSELVES.  pandoc 3.10 turns `](#7-output-
     streams)` into @ref{7 Output streams,,...}; pandoc 3.1.11 does not, and
     leaves the raw `#7-output-streams` as the target.  The container has
     3.1.11, so trusting pandoc here means the Info manual's internal
     cross-references are live or dead depending on who built it — 14 of them.
     Emitting the @ref directly makes the output identical on both, and makes
     an unresolvable anchor an error rather than a dead link.

  3. ABSOLUTISE FILE LINKS.  A relative `../README.md` resolves on GitHub and
     on disk, and is dead in Info, which has no directory to resolve it
     against.
--]]

local REPO = "https://github.com/richcreations/imud/blob/main/"

-- docs/manual.md lives in docs/, so `../x` is repo-root and `x` is docs/.
local function absolute(target)
  if target:match("^%a[%w+.-]*:") or target:match("^#") then
    return nil                                  -- already absolute, or an anchor
  end
  local anchor = target:match("(#.*)$") or ""
  local path = target:gsub("#.*$", "")
  if path == "" then return nil end
  if path:match("^%.%./") then
    path = path:gsub("^%.%./", "")
  else
    path = "docs/" .. path
  end
  return REPO .. path .. anchor
end

-- What pandoc's Texinfo writer does to a heading on its way to a @node name:
-- it drops the characters that would be read as node-list punctuation, and
-- writes an em dash the way Texinfo spells one.  Reproduced here so the @ref
-- targets this emits are the names the writer will actually produce.
local function node_name(text)
  local name = text:gsub("—", "---"):gsub("[,%.%(%)]", "")
  return (name:gsub("%s+", " "):gsub("^%s+", ""):gsub("%s+$", ""))
end

function Pandoc(doc)
  local nodes, seen = {}, {}

  doc = doc:walk({
    Header = function(el)
      local text = pandoc.utils.stringify(el.content)
      el.content = { pandoc.Str(text) }

      -- Two sections that flatten to one node name would merge in the Info
      -- file, and every @ref to either would land on whichever came first.
      if seen[text] then
        error(string.format(
          "texi-headings.lua: two headings flatten to the same node name: " ..
          "%q\n  Info addresses sections by name, so these would collide. " ..
          "Reword one.", text))
      end
      seen[text] = true
      -- Keyed by the identifier pandoc's GFM reader assigned, which is the
      -- same slug the source links to.  Matching on that rather than on a
      -- reimplementation of GitHub's slug rules is exact by construction.
      if el.identifier and el.identifier ~= "" then
        nodes[el.identifier] = node_name(text)
      end
      return el
    end,
  })

  return doc:walk({
    Link = function(el)
      local anchor = el.target:match("^#(.+)$")
      if anchor then
        local node = nodes[anchor]
        if not node then
          error(string.format(
            "texi-headings.lua: link to #%s, which is not a heading in " ..
            "docs/manual.md.\n  It would be a dead cross-reference in Info.",
            anchor))
        end
        -- @ref{NODE,,PRINTED-NAME}.  Commas separate @ref's arguments, so
        -- neither field may contain one.
        local label = pandoc.utils.stringify(el.content):gsub(",", "")
        return pandoc.RawInline(
          "texinfo", string.format("@ref{%s,,%s}", node, label))
      end
      local abs = absolute(el.target)
      if abs then el.target = abs end
      return el
    end,

    Image = function(el)
      local abs = absolute(el.src)
      if abs then el.src = abs end
      return el
    end,
  })
end
