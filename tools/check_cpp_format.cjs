const { execFileSync } = require("node:child_process");
const { readFileSync } = require("node:fs");

const [clangFormat = "clang-format", ...selectedFiles] = process.argv.slice(2);
const files = selectedFiles.length > 0
    ? selectedFiles
    : execFileSync("git", ["ls-files", "-z", "--", "src/*.cpp", "src/*.hpp"], {
        encoding: "utf8",
    }).split("\0").filter(Boolean);

const arrayInitializer =
    /^(?<indent>[ \t]*)(?<declaration>[^\r\n]+\b\w+\[[^\]\r\n]*\][ \t]*=)\n\k<indent>(?<extraIndent> +)\{\n(?<body>[\s\S]*?)^\k<indent>\k<extraIndent>\};/gm;
const closingParenBeforeBrace = /^([ \t]*\)(?: const)?)[ \t]*\n[ \t]*\{/gm;
const closingParenBeforeInitializer = /^([ \t]*\))[ \t]*\n[ \t]*(:[ \t]+)/gm;
const builderSetter =
    /^[ \t]*Builder&[ \t]+\w+\([^\n]*\)[ \t\n]*\{[^{}]*\}/gm;
const builderChain =
    /^[ \t]*[^\n]*\b\w*Builder\(\)[ \t]*\n(?=[ \t]*\.)[^;]*?;[ \t]*(?=\n|$)/gm;
const cppToken = /"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\/|->|&&|\+\+|\+=|\w+|\S/g;

function preserveBuilderLayout(source, formatted) {
    const builder = new RegExp(`${builderSetter.source}|${builderChain.source}`, "gm");
    const sourceLayouts = new Map();

    for (const [block] of source.matchAll(builder)) {
        const key = block.match(cppToken).join(" ");
        const layouts = sourceLayouts.get(key) ?? [];
        layouts.push(block);
        sourceLayouts.set(key, layouts);
    }

    return formatted.replace(builder, block =>
        sourceLayouts.get(block.match(cppToken).join(" "))?.shift() ?? block
    );
}

function findTemplateOpeners(tokens) {
    const openers = new Map();
    const stack = [];
    const closing = { ")": "(", "]": "[", "}": "{" };

    for (const [index, [token]] of tokens.entries()) {
        const previous = tokens[index - 1];
        if (token === "<" && previous &&
            (previous[0] === "template" ||
                (/^[A-Za-z_]\w*$/.test(previous[0]) &&
                    previous.index + previous[0].length === tokens[index].index))) {
            stack.push({ token, index });
        } else if (token === "(" || token === "[" || token === "{") {
            stack.push({ token, index });
        } else if (token === ">" && stack.at(-1)?.token === "<") {
            openers.set(index, stack.pop().index);
        } else if (Object.hasOwn(closing, token)) {
            while (stack.at(-1)?.token === "<") {
                stack.pop();
            }
            if (stack.at(-1)?.token === closing[token]) {
                stack.pop();
            }
        } else if (token === ";") {
            while (stack.at(-1)?.token === "<") {
                stack.pop();
            }
        }
    }
    return openers;
}

function preserveOptionalLineBreaks(source, formatted) {
    if (source === formatted) {
        return formatted;
    }

    const sourceTokens = [...source.matchAll(cppToken)];
    const formattedTokens = [...formatted.matchAll(cppToken)];
    if (sourceTokens.length !== formattedTokens.length ||
        sourceTokens.some(([token], index) => token !== formattedTokens[index][0])) {
        return formatted;
    }

    const templateOpeners = findTemplateOpeners(formattedTokens);
    let result = formatted;
    for (let index = formattedTokens.length - 1; index > 0; index--) {
        const [access] = formattedTokens[index];
        const nameIndex = formattedTokens[index + 1]?.[0] === "template" ? index + 2 : index + 1;
        const name = formattedTokens[nameIndex]?.[0];
        const next = formattedTokens[nameIndex + 1]?.[0];
        const method = (access === "." || access === "->") &&
            /^[A-Za-z_]\w*$/.test(name ?? "") && (next === "(" || next === "<");
        const opener = templateOpeners.get(index);
        const operator = (access === "&&" || access === "+") && index + 1 < formattedTokens.length;
        if (!method && opener === undefined && !operator) {
            continue;
        }

        const previousSource = sourceTokens[index - 1];
        const previousFormatted = formattedTokens[index - 1];
        const sourceGap = source.slice(previousSource.index + previousSource[0].length, sourceTokens[index].index);
        const start = previousFormatted.index + previousFormatted[0].length;
        const end = formattedTokens[index].index;
        const templateBreak = opener !== undefined && sourceGap.includes("\n") &&
            source.slice(sourceTokens[opener].index + 1, sourceTokens[opener + 1].index).includes("\n");
        if (templateBreak || (method && (sourceGap.includes("\n") || formatted.slice(start, end).includes("\n")))) {
            result = result.slice(0, start) + sourceGap + result.slice(end);
        }
        if (operator && /^\n +$/.test(sourceGap)) {
            const sourceAfter = source.slice(sourceTokens[index].index + access.length, sourceTokens[index + 1].index);
            const afterStart = end + access.length;
            const afterEnd = formattedTokens[index + 1].index;
            if (sourceAfter === " " && start < end && afterStart < afterEnd) {
                result = result.slice(0, start) + sourceGap + access + sourceAfter + result.slice(afterEnd);
            }
        }
    }
    return result;
}

function indentTernaryBranches(formatted) {
    const lines = formatted.split("\n");
    for (let index = 1; index + 1 < lines.length; index++) {
        const question = lines[index].match(/^([ ]*)\? /);
        if (!question || !lines[index + 1].startsWith(question[1] + ": ")) {
            continue;
        }

        let conditionIndex = index - 1;
        let indent = lines[conditionIndex].match(/^ */)[0];
        if (indent && lines[conditionIndex].trim() === ")") {
            while (conditionIndex > 0 && lines[conditionIndex].startsWith(indent)) {
                conditionIndex--;
            }
            indent = lines[conditionIndex].match(/^ */)[0];
        }

        lines[index] = indent + "    " + lines[index].trimStart();
        lines[index + 1] = indent + "    " + lines[index + 1].trimStart();
    }
    return lines.join("\n");
}

for (const file of files) {
    const source = readFileSync(file, "utf8").replace(/\r\n/g, "\n");
    const formatted = execFileSync(clangFormat, ["--style=file", "--Werror", file], {
        encoding: "utf8",
    }).replace(/\r\n/g, "\n");

    // The project aligns multiline array braces with the declaration.
    const projectStyle = formatted.replace(arrayInitializer, (...match) => {
        const { indent, declaration, extraIndent, body } = match.at(-1);
        const bodyIndent = indent + extraIndent;
        const alignedBody = body.split("\n").map(line =>
            line.startsWith(bodyIndent) ? indent + line.slice(bodyIndent.length) : line
        ).join("\n");
        return `${indent}${declaration}\n${indent}{\n${alignedBody}${indent}};`;
    }).replace(closingParenBeforeBrace, "$1 {")
        .replace(closingParenBeforeInitializer, "$1 $2");
    const expected = preserveOptionalLineBreaks(
        source,
        preserveBuilderLayout(source, indentTernaryBranches(projectStyle))
    );

    if (source !== expected) {
        const sourceLines = source.split("\n");
        const expectedLines = expected.split("\n");
        const firstDifference = sourceLines.findIndex((line, index) =>
            line !== expectedLines[index]
        );
        const lineNumber = firstDifference === -1 ? sourceLines.length : firstDifference + 1;
        process.stderr.write(`${file}:${lineNumber}: error: C++ formatting differs from the project style\n`);
        process.exitCode = 1;
    }
}
