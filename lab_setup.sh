#!/bin/bash

# Base command
BASE_CMD="python main.py --gpu A100a --ngpu 1 --ffopt --pipeopt --batch 1"

# Systems to test with their corresponding PIM and numhbm values
declare -A SYSTEMS=(
    ["dgx"]="none 0"
    ["dgx-attacc"]="bank 2"
    ["dgx-neurosim"]="digital 1"
)

# Models to test
MODELS=("LLAMA-7B")

# Input sequence lengths and corresponding output lengths
LIN_LENS=("128 1024 1920" "256 2048 3840")

# Batch sizes
BATCHES=(1 4 8 16)

# Loop through each system configuration
for system in "${!SYSTEMS[@]}"; do
    IFS=' ' read -r pim numhbm <<< "${SYSTEMS[$system]}"
    
    # Skip PIM-related parameters for dgx system
    if [ "$system" == "dgx" ]; then
        PIM_PARAMS=""
    else
        PIM_PARAMS="--pim $pim --numattacc 1 --numhbm $numhbm"
    fi
    
    # Loop through each model
    for model in "${MODELS[@]}"; do
        # Loop through each set of sequence lengths
        for i in "${!LIN_LENS[@]}"; do
            IFS=' ' read -r -a lins <<< "${LIN_LENS[$i]}"
            
            # Calculate lout based on whether we're in the first or second group
            if [ $i -eq 0 ]; then
                target=2048
            else
                target=4096
            fi
            
            # Loop through each input length in this group
            for lin in "${lins[@]}"; do
                lout=$((target - lin))
                
                # Loop through each batch size
                for batch in "${BATCHES[@]}"; do
                    # Construct the full command
                    CMD="$BASE_CMD --system $system $PIM_PARAMS --model $model --lin $lin --lout $lout --batch $batch"
                    
                    # Print the command (for debugging)
                    echo "Running: $CMD"
                    
                    # Execute the command
                    eval $CMD
                    
                    # Add a separator between runs
                    echo "--------------------------------------------------"
                done
            done
        done
    done
done

echo "All experiments completed!"