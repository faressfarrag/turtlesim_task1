# 🧠 SSH & File Transfer Guide (Ubuntu)

This guide explains how to securely connect to another device using **SSH** and how to transfer files between devices using **SCP**.

---

## 🔐 SSH CONNECTION METHODS

### **1️⃣ First Method (Password Authentication)**

1. Open your terminal.
2. Connect to the remote device using:
   ```bash
   ssh username@ipaddress
   ```
3. When prompted, enter the password for the remote user.

✅ **Example:**
```bash
ssh faress@192.168.1.10
```

---

### **2️⃣ Second Method (Using SSH Key Pairs)**

This method allows you to connect **without typing the password every time**.

#### A. Generate SSH Key Pair
```bash
ssh-keygen
```
- When asked for the file name, you can press **Enter** to accept the default  
  or type your own path, e.g.:
  ```
  /home/your-username/.ssh/my-key
  ```
- Press **Enter** for all prompts to skip setting a passphrase.

#### B. Copy the Public Key to Remote Device
```bash
ssh-copy-id -i ~/.ssh/my-key.pub username@ipaddress
```
- Enter the remote password once (this installs your public key on the remote device).

#### C. Connect to the Remote Device
```bash
ssh username@ipaddress
```
Now you’ll connect **immediately** without needing to type a password.

---

## 📂 COPYING FILES BETWEEN DEVICES (SCP COMMAND)

SCP (Secure Copy Protocol) allows you to copy files and directories between devices securely.

### **Syntax**
```bash
scp <source> <destination>
```

---

### **1️⃣ Copy a File from B → A (while logged into B)**
```bash
scp /path/to/file username@a:/path/to/destination
```

### **2️⃣ Copy a File from B → A (while logged into A)**
```bash
scp username@b:/path/to/file /path/to/destination
```

### **3️⃣ Copy a Directory**
Add the `-r` option to copy entire folders:
```bash
scp -r <source> <destination>
```

---

## 💡 Tips
- To verify SSH connection:
  ```bash
  ssh -T username@ipaddress
  ```
- Default SSH keys are stored in:
  ```
  ~/.ssh/
  ```
- You can use `scp` or `rsync` for more advanced syncing.

---

### ✅ Example Workflow

1. Generate your SSH key:
   ```bash
   ssh-keygen
   ```
2. Copy the public key:
   ```bash
   ssh-copy-id username@192.168.1.10
   ```
3. Connect directly:
   ```bash
   ssh username@192.168.1.10
   ```
4. Copy a file:
   ```bash
   scp myfile.txt username@192.168.1.10:/home/username/
   ```

---

📘 **Author:** Faress Farrag  
💻 **System:** Ubuntu Linux  
🔗 **Topics:** SSH, SCP, Secure File Transfer
