const express = require("express");
const nodemailer = require("nodemailer");
const multer = require("multer");
require("dotenv").config();
const bodyParser = require("body-parser");
const cors = require("cors");

const app = express();
const upload = multer(); 
const port = process.env.PORT;

app.use(cors());
app.use(bodyParser.json({ limit: '100mb' }));

// Cấu hình transporter gửi email qua Gmail (dùng app password)
const transporter = nodemailer.createTransport({
  service: 'gmail',
  auth: {
    user: process.env.USER,   // email
    pass: process.env.PASSWORD // app password
  }
});

app.post("/send", upload.single("image"), async (req, res) => {
  const { message } = req.body;
  const imageBuffer = req.file?.buffer;
  const color = message === "Cảnh báo: Có người lạ mở cửa!" ? "red" : "green";

  if (!imageBuffer || !message) {
    return res.status(400).json({ error: "Thiếu ảnh hoặc nội dung." });
  }

  try {
    await transporter.sendMail({
      from: `"ESP32-CAM" <${process.env.EMAIL_USER}>`,
      to: "quynhthom2003xxx@gmail.com",
      subject: "ESP32-CAM: Cảnh báo từ thiết bị",
      html: `
        <p><strong>Thông báo:</strong></p>
        <p style="color: ${color}; font-weight: bold; font-size: 18px;">${message}</p>
        <p>Ảnh chụp:</p>
        <img src="cid:imagecam"/>
      `,
      attachments: [
        {
          filename: "esp32cam.jpg",
          content: imageBuffer,
          cid: "imagecam",
        },
      ],
    });

    console.log("Email đã được gửi thành công!");
    res.json({ message: "Đã gửi email!" });
  } catch (err) {
    console.error("Lỗi gửi email:", err);
    res.status(500).json({ error: "Không thể gửi email." });
  }
});

app.listen(port, () => {
  console.log(`App listening on port ${port}`);
});