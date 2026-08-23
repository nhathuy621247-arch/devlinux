
## 1. Vì sao một cờ (flag) dùng chung giữa ISR và task phải là `volatile`?

Nếu không có `volatile`, trình biên dịch được phép coi biến đó như thể chỉ
luồng thực thi hiện tại mới có thể thay đổi nó. Với một vòng lặp như
`while (!flag) { }`, trình biên dịch thấy rằng không có gì *bên trong thân
vòng lặp, theo đúng những gì được viết ra* thay đổi `flag`, nên nó được phép
nạp `flag` vào một thanh ghi (register) một lần trước vòng lặp rồi cứ thế
quay vòng trên thanh ghi đó mãi mãi — không bao giờ đọc lại từ bộ nhớ. Điều
tương tự cũng xảy ra với một câu lệnh `if (flag)` đơn lẻ được kiểm tra lại ở
chỗ khác trong hàm: trình biên dịch có thể đưa việc đọc lên trước (hoist) hoặc
tái sử dụng kết quả đọc trước đó, vì theo máy trừu tượng của C thì không có gì
giữa hai lần đọc có thể làm thay đổi giá trị đó cả.

Việc ISR ghi vào biến thật trong bộ nhớ không giúp ích gì, bởi vì các tối ưu
hóa của trình biên dịch được quyết định lúc biên dịch bằng cách phân tích một
luồng điều khiển duy nhất. Một ngắt (interrupt), dưới góc nhìn của trình biên
dịch, là vô hình — chuẩn C không có khái niệm "hàm này có thể bị ngắt một
cách bất đồng bộ và ngắt đó có thể ghi vào địa chỉ này." Vì vậy trình biên
dịch áp dụng đúng tối ưu hóa mà nó sẽ áp dụng nếu biến thực sự không hề bị
đụng tới, và task có thể quay vòng mãi mãi đọc một giá trị đã lưu (cache) cũ
dù cho vị trí bộ nhớ thực tế đã được ISR cập nhật. `volatile` báo cho trình
biên dịch bỏ giả định đó: mọi lần đọc và ghi biến đều phải đi tới bộ nhớ thật,
chứ không phải một bản sao đã lưu — đó chính xác là điều cần thiết cho một
giá trị có thể thay đổi ngoài tầm quan sát của trình biên dịch.

## 2. Vì sao gọi `ESP_LOGI()` hay `vTaskDelay()` bên trong `button_isr()` lại nguy hiểm?

`ESP_LOGI()` cuối cùng sẽ định dạng một chuỗi (kiểu như `vsnprintf`) rồi ghi
nó ra UART thông qua driver, việc này có thể chiếm một mutex và có thể bị
block (chặn lại chờ). Việc chiếm mutex và bị block chỉ hợp lệ trong ngữ cảnh
task (task context) — mutex có khái niệm "task đang sở hữu" có thể bị đưa vào
trạng thái chờ rồi sau đó được đánh thức lại bởi scheduler, còn ISR thì không
có ngữ cảnh task nào để mà tạm dừng. Gọi hàm này từ ISR có nguy cơ làm hỏng
trạng thái nội bộ của FreeRTOS hoặc làm hệ thống bị treo cứng nếu mutex đó
đang bị giữ bởi một task có độ ưu tiên thấp hơn mà chính ISR vừa ngắt ngang.
Thêm vào đó, phần lớn đường đi của việc log là mã lệnh thông thường nằm trong
flash, không phải `IRAM_ATTR`; nếu ISR nổ ra đúng lúc bộ nhớ đệm (cache) của
flash đang tạm thời bị tắt (ví dụ khi đang ghi flash), việc nhảy vào đoạn mã
không nằm trong IRAM từ ISR sẽ gây ra lỗi lấy lệnh (instruction-fetch fault)
và làm sập hệ thống — đây cũng chính là lý do vì sao `button_isr()` và mọi
thứ nó gọi trực tiếp phải luôn nằm trong IRAM.

`vTaskDelay()` thậm chí còn sai một cách trực tiếp hơn: nó gọi scheduler và
tự nguyện chặn task đang gọi cho tới một mốc tick nhất định. Một ISR không
phải là một task và không có ngữ cảnh scheduler nào để mà nhường quyền thực
thi — cơ chế nó cần (đánh dấu một TCB là đang bị chặn, chọn một task khác để
chạy) không áp dụng được trên ngăn xếp ngắt (interrupt stack). Ngoài việc về
mặt kỹ thuật là không hợp lệ, nó còn giữ CPU ở mức ưu tiên ngắt trong suốt
thời gian delay, điều này chặn đứng mọi ngắt ở mức ưu tiên đó hoặc thấp hơn
trên toàn hệ thống, chứ không chỉ riêng đoạn mã bạn định tạm dừng. Đó mới là
cơ chế thực sự gây nguy hiểm — việc khóa cả một mức ưu tiên và dùng sai API
của scheduler — chứ không đơn thuần chỉ là "nó chạy chậm."

## 3. So sánh hai cách triển khai

**Độ phản hồi.** Phiên bản polling chỉ phản ứng theo chu kỳ quét 5 ms
(`CHU_KY_QUET_MS`), nên trong trường hợp xấu nhất một canh (edge) thật sự có
thể bị "ngồi im" không được phát hiện trong gần 5 ms trước khi code nhận ra.
Phiên bản dùng ngắt phản ứng ngay khi GPIO matrix kéo đường tín hiệu ngắt lên
và lõi CPU nhảy vào ISR — cỡ vài micro giây, chủ yếu do chi phí vào ngắt
(interrupt-entry overhead) chứ không phụ thuộc vào bất kỳ chu kỳ polling nào.
Đó là mức cải thiện độ trễ phát hiện trong trường hợp xấu nhất khoảng 1000
lần, và nó không bị kém đi ngay cả khi chu kỳ quét có được nới lỏng vì lý do
tiết kiệm năng lượng.

**Mức tiêu thụ CPU khi rảnh.** Task polling luôn thức dậy mỗi 5 ms, đọc
`GPIO_IN_REG`, kiểm tra dấu thời gian, rồi lại ngủ tiếp — một chi phí nhỏ
nhưng liên tục, phải trả ngay cả khi nút không hề bị đụng tới suốt cả tiếng
đồng hồ. Task của phiên bản dùng ngắt gọi `xQueueReceive()` với
`portMAX_DELAY` bất cứ khi nào không có cử chỉ nào đang diễn ra, điều này
khiến task bị block hoàn toàn; scheduler không cấp CPU trở lại cho nó cho tới
khi có một canh thật sự hoặc một deadline mà nó tự đặt ra (nguong long-press,
tick lặp lại, cửa sổ double-click) xảy đến. Mức tiêu thụ CPU khi rảnh gần như
bằng 0.

**Độ dễ làm đúng.** Thành thật mà nói, phiên bản polling dễ làm đúng hơn.
Toàn bộ trạng thái của nó nằm trong một hàm chạy trong một ngữ cảnh duy nhất,
nên logic debounce và giải mã cử chỉ chỉ đơn giản là suy luận tuần tự về
"chuyện gì đã xảy ra kể từ lần kiểm tra gần nhất." Phiên bản dùng ngắt có
cùng một cây quyết định như vậy, nhưng giờ nó bị tách ra làm hai ngữ cảnh
(ISR và task) với những quy tắc khác nhau về việc gọi hàm gì là hợp lệ, đòi
hỏi phải chọn đúng API `*FromISR` của FreeRTOS, và thay chu kỳ poll cố định
bằng một timeout hàng đợi được tính tay, phải theo dõi bất kỳ deadline nào
đang hoạt động tại thời điểm đó. Không có phần nào trong số đó khó về mặt
khái niệm cả, nhưng có nhiều chỗ hơn để làm sai phần "ống nước" (plumbing)
— quên một trường hợp biên khi debounce, tính sai timeout, hoặc gọi mã không
nằm trong IRAM từ ISR — so với một vòng lặp polling chạy tuần tự, thẳng
một mạch.
