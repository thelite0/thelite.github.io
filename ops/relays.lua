local relays = {}

for _, name in ipairs(peripheral.getNames()) do
    local isRelay = false

    if peripheral.hasType then
        isRelay = peripheral.hasType(name, "redstone_relay")
    else
        isRelay = peripheral.getType(name) == "redstone_relay"
    end

    if isRelay then
        table.insert(relays, name)
    end
end

table.sort(relays)

print("Relays found: " .. #relays)
for i, name in ipairs(relays) do
    print(i .. ". " .. name)
end

if #relays == 0 then
    print("No redstone relays visible on the peripheral network.")
end
