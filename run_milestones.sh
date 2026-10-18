#!/bin/bash

# Configuration
PLAN_FILE="plan.md"
GOAL_PROMPT="Read $PLAN_FILE, identify the first uncompleted milestone starting from M2, implement it, verify its gate and tests, commit it via git with a detailed message, mark it done in $PLAN_FILE, and repeat until M5 is complete."

echo "🚀 Starting automated Claude Code milestone pipeline..."

while true; do
    echo "🤖 Launching Claude Code with goal..."
    
    # Run Claude Code, merging stderr into stdout so we can catch rate limits,
    # and use --auto-approve to allow headless edits and test executions.
    OUTPUT=$(claude --permission-mode bypassPermissions -c "$GOAL_PROMPT" 2>&1)
    EXIT_CODE=$?
    
    # Print output to terminal so you can monitor live progress
    echo "$OUTPUT"
    
    # Check if Claude completed the goal successfully
    if [ $EXIT_CODE -eq 0 ]; then
        echo "✅ Pipeline successfully completed all remaining milestones!"
        break
    fi

    echo "⚠️ Claude Code exited with code $EXIT_CODE. Checking for rate limits..."

    # Scenario A: Absolute reset time matched (e.g., "resets 3am", "resets 14:30")
    if [[ "$OUTPUT" =~ resets[[:space:]]+([0-9]+(:[0-9]+)?(am|pm)?) ]]; then
        RESET_TIME="${BASH_REMATCH[1]}"
        echo "🛑 Hit 5-hour limit wall. Reset scheduled for: $RESET_TIME"
        
        # Calculate seconds to sleep until target absolute time (works on macOS and Linux)
        if date -d "today $RESET_TIME" &>/dev/null; then
            TARGET_SEC=$(date -d "today $RESET_TIME" +%s)
        else
            TARGET_SEC=$(date -j -f "%I%p" "$RESET_TIME" +%s 2>/dev/null || date -j -f "%H:%M" "$RESET_TIME" +%s)
        fi
        
        CURRENT_SEC=$(date +%s)
        SLEEP_DURATION=$((TARGET_SEC - CURRENT_SEC))
        
        # If the reset time is technically tomorrow morning (negative value), add 24 hours
        if [ $SLEEP_DURATION -le 0 ]; then
            SLEEP_DURATION=$((SLEEP_DURATION + 86400))
        fi

    # Scenario B: Relative reset duration matched (e.g., "resets in 3hr 3min" or "resets in 45m")
    elif [[ "$OUTPUT" =~ resets[[:space:]]+in[[:space:]]+(([0-9]+)hr)?[[:space:]]*(([0-9]+)min|[0-9]+m)? ]]; then
        HOURS=${BASH_REMATCH[2]:-0}
        MINUTES=${BASH_REMATCH[4]:-${BASH_REMATCH[3]:-0}}
        # Clean up any trailing 'm' characters if regex caught a raw short string
        MINUTES=$(echo "$MINUTES" | tr -d 'm')
        
        SLEEP_DURATION=$(( (HOURS * 3600) + (MINUTES * 60) ))
        echo "🛑 Hit rolling usage limit. Resetting in ${HOURS}h ${MINUTES}m."

    # Scenario C: Standard API Token Bucket 429 Throttle Gate
    elif [[ "$OUTPUT" =~ "Rate limit reached" || "$OUTPUT" =~ "429" ]]; then
        echo "⏳ Hit short-term API bucket rate limit. Cool-down triggered."
        SLEEP_DURATION=60 # Standard per-minute buffer
        
    else
        echo "❌ Exited due to a hard bug, crash, or manual cancellation. Stopping script."
        exit $EXIT_CODE
    fi

    # Pad sleep duration with a 30-second buffer to ensure the remote clock has rolled over
    SLEEP_DURATION=$((SLEEP_DURATION + 30))
    
    echo "💤 Sleeping for $SLEEP_DURATION seconds... The loop will automatically resume."
    sleep $SLEEP_DURATION
done
