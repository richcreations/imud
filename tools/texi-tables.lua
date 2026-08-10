--[[
  imud — IMU daemon
  Copyright (c) 2026 Richard Simpson
  SPDX-License-Identifier: MIT

  texi-tables.lua — give every table column widths, so Texinfo gets
  @columnfractions instead of a prototype row.

  A GFM pipe table carries no column widths, so pandoc emits @multitable with
  PROTOTYPE ROWS — sample cells whose printed width makeinfo uses to size the
  column:

      @multitable {@code{imud-prometheus}} {Bridge daemon (optional package):
      MQTT topics + Home Assistant discovery (see @ref{9a Bridges,,§9a
      Bridges}).}

  For docs/manual.md §4 the prototype cell runs past 300 characters, and
  makeinfo sizes the column from it: the largest section of the manual comes
  out unreadable.  It is also invalid Texinfo — a @ref may not appear on a
  @multitable line, which is the one warning makeinfo raises on the
  unfiltered conversion.

  Assigning relative widths makes pandoc emit `@multitable @columnfractions`
  instead, with no prototype row at all.

  The widths are measured from the content, and from its MAXIMUM rather than
  its mean.  makeinfo does not wrap inside a column that is too narrow — it
  runs the cells together, so a config row came out as

      'driver'string'"ism330dhcx"'Driver to load.

  with no gap anywhere.  So each column is given what its widest cell
  actually needs, and the single widest column absorbs whatever is left.
  That is how a person would size the table, and it is why the narrow columns
  cannot be normalised against a 300-character Description: doing so gives
  them a share of nothing.
--]]

local LINE = 78         -- Info's text width, which is what a fraction is of
local NARROW = 0.34     -- no non-widest column may take more than this
local WIDEST = 0.25     -- ...so the widest always keeps at least this

-- Above this many columns there is no width assignment that works: 78
-- characters split seven ways gives every column about nine, and the driver
-- table came out as
--
--     'ism330dhcx'ST   IMU        0x6A-0x6BBCM 17  *yes* --
--
-- with three pairs of cells touching.  Such a table is turned into a
-- description list instead — one entry per row, each field on its own line,
-- which is both readable and what @table gives Info to navigate by.
local MAX_COLS = 4

-- Info renders @code{x} as ‘x’, @emph{x} as _x_ and @strong{x} as *x*, so
-- each of those is two characters WIDER on screen than its text.  Measuring
-- the text alone made the Default column one short of `"ism330dhcx"` and its
-- neighbour ran straight into it.
local DECORATED = { Code = true, Emph = true, Strong = true }

local function width_of(cell)
  local div = pandoc.Div(cell.contents)
  local width = #pandoc.utils.stringify(div)
  pandoc.walk_block(div, {
    Inline = function(el)
      if DECORATED[el.tag] then width = width + 2 end
    end,
  })
  return width
end

-- The header row's cells, as inline lists, for labelling a description list.
local function headers(tbl, ncols)
  local out = {}
  for i = 1, ncols do out[i] = nil end
  local row = tbl.head.rows[1]
  if not row then return nil end
  for i, cell in ipairs(row.cells) do
    if i <= ncols then out[i] = pandoc.utils.stringify(pandoc.Div(cell.contents))
    end
  end
  return out
end

-- Row -> `@item <first cell>` with the remaining columns as `Label: value`
-- lines beneath it.  Nothing is dropped; it is the same content down the page
-- instead of across it.
local function as_description_list(tbl, ncols)
  local labels = headers(tbl, ncols)
  if not labels then return nil end

  local items = {}
  for _, body in ipairs(tbl.bodies) do
    for _, row in ipairs(body.body) do
      local term = row.cells[1] and row.cells[1].contents or {}
      local lines = {}
      for i = 2, ncols do
        local cell = row.cells[i]
        local text = cell and pandoc.utils.stringify(pandoc.Div(cell.contents))
        if text and text ~= "" then
          local inlines = { pandoc.Strong({ pandoc.Str(labels[i] or "") }),
                            pandoc.Str(":"), pandoc.Space() }
          for _, blk in ipairs(cell.contents) do
            if blk.t == "Plain" or blk.t == "Para" then
              for _, il in ipairs(blk.content) do table.insert(inlines, il) end
            end
          end
          table.insert(lines, { pandoc.Plain(inlines) })
        end
      end
      table.insert(items, {
        pandoc.utils.blocks_to_inlines(term),
        { { pandoc.BulletList(lines) } },
      })
    end
  end
  if #items == 0 then return nil end
  return pandoc.DefinitionList(items)
end

function Table(tbl)
  local ncols = #tbl.colspecs
  if ncols == 0 then return nil end

  if ncols > MAX_COLS then
    local dl = as_description_list(tbl, ncols)
    if dl then return dl end
  end

  local need = {}
  for i = 1, ncols do need[i] = 0 end

  local function measure(rows)
    for _, row in ipairs(rows) do
      for i, cell in ipairs(row.cells) do
        if i <= ncols and width_of(cell) > need[i] then
          need[i] = width_of(cell)
        end
      end
    end
  end

  for _, r in ipairs(tbl.head.rows) do measure({ r }) end
  for _, body in ipairs(tbl.bodies) do measure(body.body) end

  -- The widest column is the one that gets wrapped; every other column is
  -- given room for its longest cell plus a separating space.
  local widest = 1
  for i = 2, ncols do
    if need[i] > need[widest] then widest = i end
  end

  local fractions, used = {}, 0
  for i = 1, ncols do
    if i ~= widest then
      local f = (need[i] + 1) / LINE
      if f > NARROW then f = NARROW end
      fractions[i] = f
      used = used + f
    end
  end

  local rest = 1 - used
  if rest < WIDEST then
    -- Too many wide-ish columns to satisfy everyone: shrink them all
    -- proportionally rather than leave the wrapped column unreadable.
    local scale = (1 - WIDEST) / used
    for i = 1, ncols do
      if i ~= widest then fractions[i] = fractions[i] * scale end
    end
    rest = WIDEST
  end
  fractions[widest] = rest

  for i = 1, ncols do
    tbl.colspecs[i] = { tbl.colspecs[i][1], fractions[i] }
  end
  return tbl
end
