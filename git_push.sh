#!/bin/bash

# Step 1: Check git status
echo "📦 Checking status..."
git status

# Step 2: Add all changes
echo "➕ Staging all files..."
git add .

# Step 3: Ask for commit message
echo -n "✍️  Commit message: "
read msg

# Step 4: Commit
git commit -m "$msg"

# Step 5: Push to GitHub
echo "⬆️  Pushing to GitHub..."
git push

echo "✅ Done!"
