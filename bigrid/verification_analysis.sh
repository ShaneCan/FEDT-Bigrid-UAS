#!/usr/bin/env bash

# ============================================
# Bigraph verification analysis script
# For the paper "Bigrid-Bigraph multi-UAV collision-free navigation"
# ============================================

MODEL_FILE="uavs_in_3d_bigrid.big"
QUERIES_PROPS_FILE="verification_queries.props"

# State-space bound from the command line, default 2000
MAX_STATES=${1:-2000}

OUTPUT_DIR="verification_results/${MAX_STATES}"
TRA_FILE="${OUTPUT_DIR}/uavs_3d.tra"
CSL_FILE="${OUTPUT_DIR}/uavs_3d.csl"

# Create the output directory
mkdir -p "${OUTPUT_DIR}"

echo "============================================"
echo "Bigraph verification analysis (state space: ${MAX_STATES})"
echo "============================================"
echo "Model file:    ${MODEL_FILE}"
echo "Property file: ${QUERIES_PROPS_FILE}"
echo "State space:   ${MAX_STATES}"
echo "Output dir:    ${OUTPUT_DIR}"
echo ""

# Step 1: export the bigraph state space to PRISM format
echo "[1/4] Exporting bigraph state space (M=${MAX_STATES})..."
if [ ! -f "${TRA_FILE}" ] || [ ! -f "${CSL_FILE}" ]; then
    bigrapher full -M ${MAX_STATES} -p "${TRA_FILE}" -l "${CSL_FILE}" "${MODEL_FILE}" 2>&1 | tee "${OUTPUT_DIR}/export.log"

    if [ ! -f "${TRA_FILE}" ] || [ ! -f "${CSL_FILE}" ]; then
        echo "ERROR: state-space export failed!"
        exit 1
    fi

    echo "OK: state-space export complete"
else
    echo "OK: state-space files already exist, skipping export"
fi
echo ""

# Step 2: add the composite drone_at_ground label and append the property queries
echo "[2/4] Processing the PRISM CSL file..."
# Check whether the .csl file already contains the drone_at_ground label
if ! grep -q '^label "drone_at_ground"' "${CSL_FILE}"; then
    echo "Adding the composite drone_at_ground label..."
    python3 << PYTHON_SCRIPT
import re

# Read the .csl file
with open("${CSL_FILE}", 'r') as f:
    csl_content = f.read()

# Extract the state expression of every drone_at_ground_v* label
pattern = r'label "drone_at_ground_v\d+" = ([^;]+);'
matches = re.findall(pattern, csl_content)

if matches:
    # Combine all expressions
    combined = ' | '.join(matches)
    # Build the new label definition
    new_label = f'label "drone_at_ground" = {combined};'

    # Locate the last label definition
    last_label_pos = csl_content.rfind('label "')
    if last_label_pos != -1:
        # Locate the end of that definition (just after the semicolon)
        last_label_end = csl_content.find(';', last_label_pos) + 1
        # Insert the new label
        csl_content = csl_content[:last_label_end] + '\n' + new_label + '\n' + csl_content[last_label_end:]

    # Write the file back
    with open("${CSL_FILE}", 'w') as f:
        f.write(csl_content)
    print("OK: drone_at_ground label added")
else:
    print("WARNING: no drone_at_ground_v* label found")
PYTHON_SCRIPT
else
    echo "OK: drone_at_ground label already present"
fi

# Sync the property queries: always strip the old ones and re-append from the props file,
# so that the queries in the CSL stay consistent with the props file
echo "Syncing property queries into the CSL file..."
if [ -f "${CSL_FILE}" ]; then
    python3 << PYTHON_SCRIPT
import re
with open("${CSL_FILE}", 'r') as f:
    content = f.read()

content = re.sub(r'^P=\?.*$', '', content, flags=re.MULTILINE)
content = re.sub(r'^P>.*$', '', content, flags=re.MULTILINE)
content = re.sub(r'^P>=.*$', '', content, flags=re.MULTILINE)
content = re.sub(r'^R\{.*$', '', content, flags=re.MULTILINE)
content = re.sub(r'^[ \t]*const int N;[ \t]*\n', '', content, flags=re.MULTILINE)
content = re.sub(r'\n{3,}', '\n\n', content)

with open("${CSL_FILE}", 'w') as f:
    f.write(content.rstrip() + '\n')
print("OK: old queries removed")
PYTHON_SCRIPT

    # Append the query statements
    grep -E "^P=\?|^P>|^P>=|^R\{" "${QUERIES_PROPS_FILE}" | grep -v "^//" | grep -v "^const" >> "${CSL_FILE}"
    echo "OK: property queries appended"

    # Insert a const int N; declaration if any F<=N query is present
    if grep -q "F<=N" "${CSL_FILE}"; then
        python3 << PYTHON_SCRIPT
import re
with open("${CSL_FILE}", 'r') as f:
    content = f.read()

first_query = re.search(r'^(P=\?|P>|P>=|R\{)', content, re.MULTILINE)
if first_query and 'const int N;' not in content[:first_query.start()]:
    insert_pos = first_query.start()
    new_content = content[:insert_pos].rstrip() + '\n\nconst int N;\n' + content[insert_pos:]
    with open("${CSL_FILE}", 'w') as f:
        f.write(new_content)
    print("OK: const int N; declaration added")
PYTHON_SCRIPT
    fi
fi
echo ""

# Step 3: run the PRISM verification queries
echo "[3/4] Running PRISM verification queries..."

# Count the properties (skipping comments and blank lines)
TOTAL_PROPS=$(grep -E "^property|^const" "${QUERIES_PROPS_FILE}" | grep -v "^//" | wc -l)
echo "Found ${TOTAL_PROPS} properties"
echo ""

# Property numbering starts at 1
PROP_NUM=1

# Read the property file and run a query for each property
while IFS= read -r line || [ -n "$line" ]; do
    # Skip comments and blank lines
    if [[ "$line" =~ ^[[:space:]]*$ ]] || [[ "$line" =~ ^[[:space:]]*// ]]; then
        continue
    fi

    # Skip const declarations (handled inside the queries)
    if [[ "$line" =~ ^[[:space:]]*const ]]; then
        continue
    fi

    # Check whether this is a query (starts with P=?, P>0, P>=1 or R{)
    if [[ "$line" =~ ^[[:space:]]*(P=\?|P>|P>=|R\{) ]]; then
        PROP_QUERY=$(echo "$line" | xargs)

        echo "----------------------------------------"
        echo "Property ${PROP_NUM}: ${PROP_QUERY}"
        echo "----------------------------------------"

        # Check whether a const parameter is needed (matches the F<=N form)
        if echo "$PROP_QUERY" | grep -q "F<=N"; then
            # Query that needs a step-bound parameter
            prism -importtrans "${TRA_FILE}" "${CSL_FILE}" \
                -dtmc -prop ${PROP_NUM} \
                -const N=100 \
                -exportresults "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" 2>&1 | \
                tee "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt"
        # Check whether this is a global invariant query (G operator) - needs a larger stack
        elif echo "$PROP_QUERY" | grep -q "G "; then
            # Global invariant queries need more stack space; use 512m (4g exceeds the JVM limit)
            prism -importtrans "${TRA_FILE}" "${CSL_FILE}" \
                -dtmc -prop ${PROP_NUM} \
                -javastack 512m \
                -exportresults "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" 2>&1 | \
                tee "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt"
        else
            # Ordinary query
            prism -importtrans "${TRA_FILE}" "${CSL_FILE}" \
                -dtmc -prop ${PROP_NUM} \
                -exportresults "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" 2>&1 | \
                tee "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt"
        fi

        # Extract the result
        if [ -f "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" ]; then
            # PRISM result-file format:
            # - for P=? queries: line 1 is "Result", line 2 is the numeric value
            # - for P>0/P>=1 boolean queries: line 1 is "Result", line 2 is "true" or "false"
            RESULT=$(cat "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" | tail -n +2 | head -1 | xargs)
            # If there is no second line, try to extract it from the log
            if [ -z "$RESULT" ] || [ "$RESULT" = "Result" ]; then
                if [ -f "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" ]; then
                    # Try to extract a numeric result
                    RESULT=$(grep -E "Value in the initial state:|Result:" "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" | head -1 | sed 's/.*Value in the initial state: //' | sed 's/.*Result: //' | xargs)
                    # For a boolean query, try to extract true/false
                    if [ -z "$RESULT" ] || [ "$RESULT" = "Result" ]; then
                        RESULT=$(grep -E "Property.*(true|false)|(true|false)" "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" | head -1 | grep -oE "(true|false)" | head -1)
                    fi
                fi
            fi
            if [ -n "$RESULT" ] && [ "$RESULT" != "Result" ]; then
                echo "Result: ${RESULT}"
            else
                echo "Result: query failed or no result found"
            fi
        else
            echo "Result: result file not found"
        fi

        PROP_NUM=$((PROP_NUM + 1))
        echo ""
    fi
done < "${QUERIES_PROPS_FILE}"

echo "OK: all queries executed"
echo ""

# Step 4: generate the analysis report
echo "[4/4] Generating the analysis report..."
REPORT_FILE="${OUTPUT_DIR}/verification_report.md"

cat > "${REPORT_FILE}" << 'EOF'
# Bigraph verification analysis report

This report presents the verification results of the bigraph rules acting as a **safety checking
mechanism**, for the paper "Bigrid-Bigraph multi-UAV collision-free navigation".

EOF

# Re-read the query file to extract the comments and query statements
PROP_NUM=1
QUERY_DESC=""
QUERY_CONTENT=""
IN_MULTILINE_COMMENT=false

while IFS= read -r line || [ -n "$line" ]; do
    # Extract the description comment - matches the Q<digits>.<digits>: form
    if [[ "$line" =~ ^[[:space:]]*//.*Q[0-9]+\.[0-9]+:.* ]]; then
        QUERY_DESC=$(echo "$line" | sed 's/^[[:space:]]*\/\/[[:space:]]*//')
        IN_MULTILINE_COMMENT=true
    # Extract continuation comments (comment lines directly following the query comment)
    elif [[ "$line" =~ ^[[:space:]]*// ]] && [ "$IN_MULTILINE_COMMENT" = true ]; then
        COMMENT=$(echo "$line" | sed 's/^[[:space:]]*\/\/[[:space:]]*//')
        if [ -n "$COMMENT" ]; then
            if [ -z "$QUERY_DESC" ]; then
                QUERY_DESC="$COMMENT"
            else
                QUERY_DESC="${QUERY_DESC} ${COMMENT}"
            fi
        fi
    fi

    # Skip const declarations
    if [[ "$line" =~ ^[[:space:]]*const ]]; then
        continue
    fi

    # Check whether this is a query (starts with P=?, P>0, P>=1 or R{)
    if [[ "$line" =~ ^[[:space:]]*(P=\?|P>|P>=|R\{) ]]; then
        QUERY_CONTENT=$(echo "$line" | xargs)
        IN_MULTILINE_COMMENT=false

        # Write the report entry
        echo "### Query ${PROP_NUM}" >> "${REPORT_FILE}"
        echo "" >> "${REPORT_FILE}"
        if [ -n "$QUERY_DESC" ]; then
            echo "**Meaning**: ${QUERY_DESC}" >> "${REPORT_FILE}"
            echo "" >> "${REPORT_FILE}"
        fi
        echo "**Query**: \`${QUERY_CONTENT}\`" >> "${REPORT_FILE}"
        echo "" >> "${REPORT_FILE}"

        # Extract the result
        if [ -f "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" ]; then
            # PRISM result-file format:
            # - for P=? queries: line 1 is "Result", line 2 is the numeric value
            # - for P>0/P>=1 boolean queries: line 1 is "Result", line 2 is "true" or "false"
            RESULT=$(cat "${OUTPUT_DIR}/query_${PROP_NUM}_result.txt" | tail -n +2 | head -1 | xargs)
            # If there is no second line, try to extract it from the log
            if [ -z "$RESULT" ] || [ "$RESULT" = "Result" ]; then
                if [ -f "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" ]; then
                    # Try to extract a numeric result
                    RESULT=$(grep -E "Value in the initial state:|Result:" "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" | head -1 | sed 's/.*Value in the initial state: //' | sed 's/.*Result: //' | xargs)
                    # For a boolean query, try to extract true/false
                    if [ -z "$RESULT" ] || [ "$RESULT" = "Result" ]; then
                        RESULT=$(grep -E "Property.*(true|false)|(true|false)" "${OUTPUT_DIR}/query_${PROP_NUM}_log.txt" | head -1 | grep -oE "(true|false)" | head -1)
                    fi
                fi
            fi
            if [ -n "$RESULT" ] && [ "$RESULT" != "Result" ]; then
                echo "**Verification result**: ${RESULT}" >> "${REPORT_FILE}"
            else
                echo "**Verification result**: query failed or no result found" >> "${REPORT_FILE}"
            fi
        else
            echo "**Verification result**: result file not found" >> "${REPORT_FILE}"
        fi

        echo "" >> "${REPORT_FILE}"
        echo "---" >> "${REPORT_FILE}"
        echo "" >> "${REPORT_FILE}"

        # Reset the variables
        QUERY_DESC=""
        QUERY_CONTENT=""
        PROP_NUM=$((PROP_NUM + 1))
    elif [[ ! "$line" =~ ^[[:space:]]*// ]] && [ "$IN_MULTILINE_COMMENT" = true ]; then
        # A non-comment line ends the multi-line comment block
        IN_MULTILINE_COMMENT=false
    fi
done < "${QUERIES_PROPS_FILE}"


echo "============================================"
echo "Analysis complete!"
echo "============================================"
echo "Result files: ${OUTPUT_DIR}/"
echo "Report file:  ${REPORT_FILE}"
echo ""

